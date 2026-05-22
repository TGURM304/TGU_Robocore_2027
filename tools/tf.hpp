/**
 * @file tf.hpp
 * @brief 线程安全的轻量级 TF 树工具，提供类似 ROS2 tf2 的坐标系维护与查询能力。
 */

#ifndef TGU_ROBOCORE_2027_TF_HPP
#define TGU_ROBOCORE_2027_TF_HPP

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace tools {

/**
 * @brief 带时间戳的坐标变换。
 *
 * 变换语义为 parent_frame_T_child_frame：
 *
 * @code
 * p_parent = transform * p_child;
 * @endcode
 */
struct TransformStamped {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string parent_frame;
    std::string child_frame;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    uint64_t stamp_ns = 0;
    bool is_static = false;
};

class TfException : public std::runtime_error {
public:
    explicit TfException(const std::string& message);
};

class FrameNotFoundException : public TfException {
public:
    explicit FrameNotFoundException(const std::string& message);
};

class ConnectivityException : public TfException {
public:
    explicit ConnectivityException(const std::string& message);
};

class ExtrapolationException : public TfException {
public:
    explicit ExtrapolationException(const std::string& message);
};

/**
 * @brief 线程安全的 TF 缓冲区。
 *
 * 使用方式和 ROS2 tf2 的核心语义类似：
 * - set_transform(parent, child, parent_T_child, stamp_ns) 写入动态 TF。
 * - set_static_transform(parent, child, parent_T_child) 写入静态 TF。
 * - lookup_transform(target, source, stamp_ns) 查询 target_T_source。
 *
 * 坐标变换方向约定：
 * @code
 * Eigen::Isometry3d target_T_source = lookup_transform(target, source, time);
 * Eigen::Vector3d p_target = target_T_source * p_source;
 * @endcode
 *
 * 线程安全说明：
 * - 所有 public 成员函数都是线程安全的。
 * - 多个只读查询可以并发执行。
 * - 写入操作与其它读写操作互斥。
 * - public 查询结果均按值返回，不暴露内部容器、引用或迭代器。
 * - 调用者需要保证 TfBuffer 对象生命周期长于所有正在访问它的线程。
 */
class TfBuffer {
public:
    static constexpr uint64_t LATEST_TIME = 0;
    static constexpr uint64_t DEFAULT_CACHE_TIME_NS = 10'000'000'000ULL;

    explicit TfBuffer(uint64_t cache_time_ns = DEFAULT_CACHE_TIME_NS);
    ~TfBuffer() = default;

    TfBuffer(const TfBuffer&) = delete;
    TfBuffer& operator=(const TfBuffer&) = delete;
    TfBuffer(TfBuffer&&) = delete;
    TfBuffer& operator=(TfBuffer&&) = delete;

    /**
     * @brief 写入动态 TF。
     * @param parent_frame 父坐标系。
     * @param child_frame 子坐标系。
     * @param parent_T_child 从 child_frame 到 parent_frame 的变换。
     * @param stamp_ns 时间戳，单位 ns。0 被保留为 LATEST_TIME，不允许作为动态 TF 时间戳。
     * @return true 写入成功；false 输入非法、父子关系冲突或会形成环。
     */
    bool set_transform(
        const std::string& parent_frame,
        const std::string& child_frame,
        const Eigen::Isometry3d& parent_T_child,
        uint64_t stamp_ns
    );

    /**
     * @brief 写入静态 TF。
     * @param parent_frame 父坐标系。
     * @param child_frame 子坐标系。
     * @param parent_T_child 从 child_frame 到 parent_frame 的固定变换。
     * @return true 写入成功；false 输入非法、父子关系冲突或会形成环。
     */
    bool set_static_transform(
        const std::string& parent_frame,
        const std::string& child_frame,
        const Eigen::Isometry3d& parent_T_child
    );

    /**
     * @brief 查询 target_frame_T_source_frame。
     * @param target_frame 目标坐标系。
     * @param source_frame 源坐标系。
     * @param stamp_ns 查询时间戳，单位 ns。LATEST_TIME 表示查询路径上的最新公共时间。
     * @return target_frame_T_source_frame。
     * @throws TfException 查询失败时抛出具体 TF 异常。
     */
    [[nodiscard]] Eigen::Isometry3d lookup_transform(
        const std::string& target_frame,
        const std::string& source_frame,
        uint64_t stamp_ns = LATEST_TIME
    ) const;

    /**
     * @brief 判断指定 TF 是否可以查询。
     * @param target_frame 目标坐标系。
     * @param source_frame 源坐标系。
     * @param stamp_ns 查询时间戳，单位 ns。LATEST_TIME 表示查询路径上的最新公共时间。
     * @param error_msg 可选错误信息输出。
     * @return true 可以查询；false 不可查询。
     */
    [[nodiscard]] bool can_transform(
        const std::string& target_frame,
        const std::string& source_frame,
        uint64_t stamp_ns = LATEST_TIME,
        std::string* error_msg = nullptr
    ) const noexcept;

    /**
     * @brief 将点从 source_frame 变换到 target_frame，应用旋转和平移。
     */
    [[nodiscard]] Eigen::Vector3d transform_point(
        const std::string& target_frame,
        const std::string& source_frame,
        const Eigen::Vector3d& point_source,
        uint64_t stamp_ns = LATEST_TIME
    ) const;

    /**
     * @brief 将向量从 source_frame 变换到 target_frame，只应用旋转，不应用平移。
     */
    [[nodiscard]] Eigen::Vector3d transform_vector(
        const std::string& target_frame,
        const std::string& source_frame,
        const Eigen::Vector3d& vector_source,
        uint64_t stamp_ns = LATEST_TIME
    ) const;

    /**
     * @brief 将位姿从 source_frame 变换到 target_frame。
     */
    [[nodiscard]] Eigen::Isometry3d transform_pose(
        const std::string& target_frame,
        const std::string& source_frame,
        const Eigen::Isometry3d& pose_source,
        uint64_t stamp_ns = LATEST_TIME
    ) const;

    /**
     * @brief 判断 frame 是否存在于 TF 树中。
     */
    [[nodiscard]] bool has_frame(const std::string& frame) const;

    /**
     * @brief 获取当前所有 frame 名称，按字典序排序后返回。
     */
    [[nodiscard]] std::vector<std::string> get_frames() const;

    /**
     * @brief 以字符串形式输出当前 TF 树关系，主要用于调试和日志。
     */
    [[nodiscard]] std::string all_frames_as_string() const;

    /**
     * @brief 清空所有 TF 数据。
     */
    void clear();

    /**
     * @brief 设置动态 TF 缓存长度，单位 ns。
     *
     * 设置后会立即裁剪已有动态缓存。静态 TF 不受影响。
     */
    void set_cache_time(uint64_t cache_time_ns);

    /**
     * @brief 获取当前动态 TF 缓存长度，单位 ns。
     */
    [[nodiscard]] uint64_t cache_time() const;

private:
    struct TimeCache {
        std::string parent_frame;
        bool is_static = false;
        std::deque<TransformStamped> transforms;
    };

    [[nodiscard]] Eigen::Isometry3d lookup_transform_locked(
        const std::string& target_frame,
        const std::string& source_frame,
        uint64_t stamp_ns
    ) const;

    [[nodiscard]] bool has_frame_locked(const std::string& frame) const;
    [[nodiscard]] std::vector<std::string> get_frames_locked() const;

    [[nodiscard]] std::vector<std::string> get_frame_chain_locked(
        const std::string& frame
    ) const;

    [[nodiscard]] std::pair<size_t, size_t> find_common_ancestor_locked(
        const std::vector<std::string>& target_chain,
        const std::vector<std::string>& source_chain,
        const std::string& target_frame,
        const std::string& source_frame
    ) const;

    [[nodiscard]] uint64_t resolve_latest_common_time_locked(
        const std::vector<std::string>& target_chain,
        size_t target_common_index,
        const std::vector<std::string>& source_chain,
        size_t source_common_index
    ) const;

    [[nodiscard]] Eigen::Isometry3d compose_to_ancestor_locked(
        const std::vector<std::string>& chain,
        size_t ancestor_index,
        uint64_t stamp_ns
    ) const;

    [[nodiscard]] Eigen::Isometry3d get_transform_at_time_locked(
        const std::string& child_frame,
        uint64_t stamp_ns
    ) const;

    [[nodiscard]] bool can_set_transform_locked(
        const std::string& parent_frame,
        const std::string& child_frame,
        const Eigen::Isometry3d& parent_T_child,
        uint64_t stamp_ns,
        bool is_static
    ) const;

    [[nodiscard]] bool would_create_cycle_locked(
        const std::string& parent_frame,
        const std::string& child_frame
    ) const;

    void prune_old_transforms_locked(TimeCache& cache) const;

    [[nodiscard]] static Eigen::Isometry3d interpolate_transform(
        const TransformStamped& before,
        const TransformStamped& after,
        uint64_t stamp_ns
    );

    [[nodiscard]] static bool is_valid_frame_id(const std::string& frame);
    [[nodiscard]] static bool is_valid_transform(const Eigen::Isometry3d& transform);

    uint64_t cache_time_ns_ = DEFAULT_CACHE_TIME_NS;
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, TimeCache> child_to_cache_;
};

} // namespace tools

#endif // TGU_ROBOCORE_2027_TF_HPP
