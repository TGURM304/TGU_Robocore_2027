/**
 * @file tf.cpp
 * @brief 线程安全的轻量级 TF 树工具实现。
 */

#include "tf.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <mutex>
#include <sstream>
#include <unordered_set>

namespace tools {
namespace {

constexpr double MIN_VALID_DETERMINANT = 1e-12;

std::string quote_frame(const std::string& frame) {
    return "\"" + frame + "\"";
}

void safe_set_error(std::string* error_msg, const char* message) noexcept {
    if (error_msg == nullptr) {
        return;
    }

    try {
        *error_msg = (message == nullptr) ? "unknown TF error" : message;
    } catch (...) {
        // can_transform() 是 noexcept，错误字符串分配失败时只能忽略。
    }
}

void safe_clear_error(std::string* error_msg) noexcept {
    if (error_msg == nullptr) {
        return;
    }

    try {
        error_msg->clear();
    } catch (...) {
        // can_transform() 是 noexcept，错误字符串清理失败时只能忽略。
    }
}

} // namespace

TfException::TfException(const std::string& message) : std::runtime_error(message) {}

FrameNotFoundException::FrameNotFoundException(const std::string& message) : TfException(message) {}

ConnectivityException::ConnectivityException(const std::string& message) : TfException(message) {}

ExtrapolationException::ExtrapolationException(const std::string& message) : TfException(message) {}

TfBuffer::TfBuffer(uint64_t cache_time_ns) : cache_time_ns_(cache_time_ns) {}

bool TfBuffer::set_transform(
    const std::string& parent_frame,
    const std::string& child_frame,
    const Eigen::Isometry3d& parent_T_child,
    uint64_t stamp_ns
) {
    std::unique_lock lock(mutex_);

    if (!can_set_transform_locked(parent_frame, child_frame, parent_T_child, stamp_ns, false)) {
        return false;
    }

    auto cache_it = child_to_cache_.find(child_frame);
    if (cache_it == child_to_cache_.end()) {
        TimeCache cache;
        cache.parent_frame = parent_frame;
        cache.is_static = false;
        cache_it = child_to_cache_.emplace(child_frame, std::move(cache)).first;
    }

    TimeCache& cache = cache_it->second;

    TransformStamped transform;
    transform.parent_frame = parent_frame;
    transform.child_frame = child_frame;
    transform.transform = parent_T_child;
    transform.stamp_ns = stamp_ns;
    transform.is_static = false;

    auto insert_pos = std::lower_bound(
        cache.transforms.begin(),
        cache.transforms.end(),
        stamp_ns,
        [](const TransformStamped& value, uint64_t stamp) {
            return value.stamp_ns < stamp;
        }
    );

    if (insert_pos != cache.transforms.end() && insert_pos->stamp_ns == stamp_ns) {
        *insert_pos = std::move(transform);
    } else {
        cache.transforms.insert(insert_pos, std::move(transform));
    }

    prune_old_transforms_locked(cache);
    return true;
}

bool TfBuffer::set_static_transform(
    const std::string& parent_frame,
    const std::string& child_frame,
    const Eigen::Isometry3d& parent_T_child
) {
    std::unique_lock lock(mutex_);

    if (!can_set_transform_locked(parent_frame, child_frame, parent_T_child, LATEST_TIME, true)) {
        return false;
    }

    TransformStamped transform;
    transform.parent_frame = parent_frame;
    transform.child_frame = child_frame;
    transform.transform = parent_T_child;
    transform.stamp_ns = LATEST_TIME;
    transform.is_static = true;

    auto cache_it = child_to_cache_.find(child_frame);
    if (cache_it == child_to_cache_.end()) {
        TimeCache cache;
        cache.parent_frame = parent_frame;
        cache.is_static = true;
        cache.transforms.push_back(std::move(transform));
        child_to_cache_.emplace(child_frame, std::move(cache));
        return true;
    }

    TimeCache& cache = cache_it->second;
    cache.parent_frame = parent_frame;
    cache.is_static = true;
    cache.transforms.clear();
    cache.transforms.push_back(std::move(transform));
    return true;
}

Eigen::Isometry3d TfBuffer::lookup_transform(
    const std::string& target_frame,
    const std::string& source_frame,
    uint64_t stamp_ns
) const {
    std::shared_lock lock(mutex_);
    return lookup_transform_locked(target_frame, source_frame, stamp_ns);
}

bool TfBuffer::can_transform(
    const std::string& target_frame,
    const std::string& source_frame,
    uint64_t stamp_ns,
    std::string* error_msg
) const noexcept {
    try {
        std::shared_lock lock(mutex_);
        static_cast<void>(lookup_transform_locked(target_frame, source_frame, stamp_ns));
        safe_clear_error(error_msg);
        return true;
    } catch (const std::exception& e) {
        safe_set_error(error_msg, e.what());
        return false;
    } catch (...) {
        safe_set_error(error_msg, "unknown TF error");
        return false;
    }
}

Eigen::Vector3d TfBuffer::transform_point(
    const std::string& target_frame,
    const std::string& source_frame,
    const Eigen::Vector3d& point_source,
    uint64_t stamp_ns
) const {
    std::shared_lock lock(mutex_);
    const Eigen::Isometry3d target_T_source = lookup_transform_locked(
        target_frame,
        source_frame,
        stamp_ns
    );
    return target_T_source * point_source;
}

Eigen::Vector3d TfBuffer::transform_vector(
    const std::string& target_frame,
    const std::string& source_frame,
    const Eigen::Vector3d& vector_source,
    uint64_t stamp_ns
) const {
    std::shared_lock lock(mutex_);
    const Eigen::Isometry3d target_T_source = lookup_transform_locked(
        target_frame,
        source_frame,
        stamp_ns
    );
    return target_T_source.linear() * vector_source;
}

Eigen::Isometry3d TfBuffer::transform_pose(
    const std::string& target_frame,
    const std::string& source_frame,
    const Eigen::Isometry3d& pose_source,
    uint64_t stamp_ns
) const {
    std::shared_lock lock(mutex_);
    const Eigen::Isometry3d target_T_source = lookup_transform_locked(
        target_frame,
        source_frame,
        stamp_ns
    );
    return target_T_source * pose_source;
}

bool TfBuffer::has_frame(const std::string& frame) const {
    std::shared_lock lock(mutex_);
    return has_frame_locked(frame);
}

std::vector<std::string> TfBuffer::get_frames() const {
    std::shared_lock lock(mutex_);
    return get_frames_locked();
}

std::string TfBuffer::all_frames_as_string() const {
    std::shared_lock lock(mutex_);

    std::ostringstream oss;
    const std::vector<std::string> frames = get_frames_locked();

    oss << "TF frames: " << frames.size() << '\n';

    std::vector<std::string> children;
    children.reserve(child_to_cache_.size());
    for (const auto& [child, cache] : child_to_cache_) {
        static_cast<void>(cache);
        children.push_back(child);
    }
    std::sort(children.begin(), children.end());

    for (const std::string& child : children) {
        const TimeCache& cache = child_to_cache_.at(child);
        oss << "  " << cache.parent_frame << " -> " << child;

        if (cache.is_static) {
            oss << " [static]";
        } else {
            oss << " [dynamic, samples=" << cache.transforms.size();
            if (!cache.transforms.empty()) {
                oss << ", oldest=" << cache.transforms.front().stamp_ns
                    << ", latest=" << cache.transforms.back().stamp_ns;
            }
            oss << "]";
        }

        oss << '\n';
    }

    return oss.str();
}

void TfBuffer::clear() {
    std::unique_lock lock(mutex_);
    child_to_cache_.clear();
}

void TfBuffer::set_cache_time(uint64_t cache_time_ns) {
    std::unique_lock lock(mutex_);
    cache_time_ns_ = cache_time_ns;

    for (auto& [child, cache] : child_to_cache_) {
        static_cast<void>(child);
        prune_old_transforms_locked(cache);
    }
}

uint64_t TfBuffer::cache_time() const {
    std::shared_lock lock(mutex_);
    return cache_time_ns_;
}

Eigen::Isometry3d TfBuffer::lookup_transform_locked(
    const std::string& target_frame,
    const std::string& source_frame,
    uint64_t stamp_ns
) const {
    if (!is_valid_frame_id(target_frame)) {
        throw FrameNotFoundException("target frame is empty or invalid");
    }
    if (!is_valid_frame_id(source_frame)) {
        throw FrameNotFoundException("source frame is empty or invalid");
    }

    if (target_frame == source_frame) {
        return Eigen::Isometry3d::Identity();
    }

    const std::vector<std::string> target_chain = get_frame_chain_locked(target_frame);
    const std::vector<std::string> source_chain = get_frame_chain_locked(source_frame);

    const auto [target_common_index, source_common_index] = find_common_ancestor_locked(
        target_chain,
        source_chain,
        target_frame,
        source_frame
    );

    const uint64_t lookup_time = (stamp_ns == LATEST_TIME)
        ? resolve_latest_common_time_locked(
            target_chain,
            target_common_index,
            source_chain,
            source_common_index
        )
        : stamp_ns;

    const Eigen::Isometry3d common_T_target = compose_to_ancestor_locked(
        target_chain,
        target_common_index,
        lookup_time
    );
    const Eigen::Isometry3d common_T_source = compose_to_ancestor_locked(
        source_chain,
        source_common_index,
        lookup_time
    );

    return common_T_target.inverse() * common_T_source;
}

bool TfBuffer::has_frame_locked(const std::string& frame) const {
    if (!is_valid_frame_id(frame)) {
        return false;
    }

    if (child_to_cache_.find(frame) != child_to_cache_.end()) {
        return true;
    }

    return std::any_of(
        child_to_cache_.begin(),
        child_to_cache_.end(),
        [&frame](const auto& item) {
            return item.second.parent_frame == frame;
        }
    );
}

std::vector<std::string> TfBuffer::get_frames_locked() const {
    std::unordered_set<std::string> unique_frames;

    for (const auto& [child, cache] : child_to_cache_) {
        unique_frames.insert(child);
        unique_frames.insert(cache.parent_frame);
    }

    std::vector<std::string> frames(unique_frames.begin(), unique_frames.end());
    std::sort(frames.begin(), frames.end());
    return frames;
}

std::vector<std::string> TfBuffer::get_frame_chain_locked(const std::string& frame) const {
    if (!has_frame_locked(frame)) {
        throw FrameNotFoundException("frame " + quote_frame(frame) + " does not exist");
    }

    std::vector<std::string> chain;
    std::unordered_set<std::string> visited;
    std::string current = frame;

    while (true) {
        if (!visited.insert(current).second) {
            throw ConnectivityException("cycle detected while walking frame " + quote_frame(frame));
        }

        chain.push_back(current);

        const auto cache_it = child_to_cache_.find(current);
        if (cache_it == child_to_cache_.end()) {
            break;
        }

        if (!is_valid_frame_id(cache_it->second.parent_frame)) {
            throw TfException("invalid parent frame stored for child " + quote_frame(current));
        }

        current = cache_it->second.parent_frame;
    }

    return chain;
}

std::pair<size_t, size_t> TfBuffer::find_common_ancestor_locked(
    const std::vector<std::string>& target_chain,
    const std::vector<std::string>& source_chain,
    const std::string& target_frame,
    const std::string& source_frame
) const {
    for (size_t target_index = 0; target_index < target_chain.size(); ++target_index) {
        const std::string& target_ancestor = target_chain[target_index];

        for (size_t source_index = 0; source_index < source_chain.size(); ++source_index) {
            if (source_chain[source_index] == target_ancestor) {
                return {target_index, source_index};
            }
        }
    }

    throw ConnectivityException(
        "frames " + quote_frame(target_frame) + " and " + quote_frame(source_frame) +
        " are not connected"
    );
}

uint64_t TfBuffer::resolve_latest_common_time_locked(
    const std::vector<std::string>& target_chain,
    size_t target_common_index,
    const std::vector<std::string>& source_chain,
    size_t source_common_index
) const {
    uint64_t latest_common_time = std::numeric_limits<uint64_t>::max();
    bool has_dynamic_edge = false;

    const auto update_latest_common_time = [this, &latest_common_time, &has_dynamic_edge](
        const std::vector<std::string>& chain,
        size_t common_index
    ) {
        for (size_t index = 0; index < common_index; ++index) {
            const std::string& child = chain[index];
            const auto cache_it = child_to_cache_.find(child);
            if (cache_it == child_to_cache_.end()) {
                throw FrameNotFoundException("frame " + quote_frame(child) + " does not have parent TF");
            }

            const TimeCache& cache = cache_it->second;
            if (cache.is_static) {
                continue;
            }
            if (cache.transforms.empty()) {
                throw ExtrapolationException("dynamic TF cache for child " + quote_frame(child) + " is empty");
            }

            has_dynamic_edge = true;
            latest_common_time = std::min(latest_common_time, cache.transforms.back().stamp_ns);
        }
    };

    update_latest_common_time(target_chain, target_common_index);
    update_latest_common_time(source_chain, source_common_index);

    if (!has_dynamic_edge) {
        return LATEST_TIME;
    }

    return latest_common_time;
}

Eigen::Isometry3d TfBuffer::compose_to_ancestor_locked(
    const std::vector<std::string>& chain,
    size_t ancestor_index,
    uint64_t stamp_ns
) const {
    Eigen::Isometry3d ancestor_T_frame = Eigen::Isometry3d::Identity();

    for (size_t index = 0; index < ancestor_index; ++index) {
        const std::string& child = chain[index];
        const Eigen::Isometry3d parent_T_child = get_transform_at_time_locked(child, stamp_ns);
        ancestor_T_frame = parent_T_child * ancestor_T_frame;
    }

    return ancestor_T_frame;
}

Eigen::Isometry3d TfBuffer::get_transform_at_time_locked(
    const std::string& child_frame,
    uint64_t stamp_ns
) const {
    const auto cache_it = child_to_cache_.find(child_frame);
    if (cache_it == child_to_cache_.end()) {
        throw FrameNotFoundException("frame " + quote_frame(child_frame) + " does not have parent TF");
    }

    const TimeCache& cache = cache_it->second;
    if (cache.transforms.empty()) {
        throw ExtrapolationException("TF cache for child " + quote_frame(child_frame) + " is empty");
    }

    if (cache.is_static || stamp_ns == LATEST_TIME) {
        return cache.transforms.back().transform;
    }

    if (stamp_ns < cache.transforms.front().stamp_ns) {
        throw ExtrapolationException(
            "requested time " + std::to_string(stamp_ns) + " is earlier than oldest TF " +
            std::to_string(cache.transforms.front().stamp_ns) + " for child " + quote_frame(child_frame)
        );
    }

    if (stamp_ns > cache.transforms.back().stamp_ns) {
        throw ExtrapolationException(
            "requested time " + std::to_string(stamp_ns) + " is later than latest TF " +
            std::to_string(cache.transforms.back().stamp_ns) + " for child " + quote_frame(child_frame)
        );
    }

    const auto lower = std::lower_bound(
        cache.transforms.begin(),
        cache.transforms.end(),
        stamp_ns,
        [](const TransformStamped& value, uint64_t stamp) {
            return value.stamp_ns < stamp;
        }
    );

    if (lower != cache.transforms.end() && lower->stamp_ns == stamp_ns) {
        return lower->transform;
    }

    if (lower == cache.transforms.begin() || lower == cache.transforms.end()) {
        throw ExtrapolationException(
            "cannot interpolate TF for child " + quote_frame(child_frame) +
            " at time " + std::to_string(stamp_ns)
        );
    }

    const auto before = std::prev(lower);
    return interpolate_transform(*before, *lower, stamp_ns);
}

bool TfBuffer::can_set_transform_locked(
    const std::string& parent_frame,
    const std::string& child_frame,
    const Eigen::Isometry3d& parent_T_child,
    uint64_t stamp_ns,
    bool is_static
) const {
    if (!is_valid_frame_id(parent_frame) || !is_valid_frame_id(child_frame)) {
        return false;
    }
    if (parent_frame == child_frame) {
        return false;
    }
    if (!is_valid_transform(parent_T_child)) {
        return false;
    }
    if (!is_static && stamp_ns == LATEST_TIME) {
        return false;
    }

    const auto cache_it = child_to_cache_.find(child_frame);
    if (cache_it != child_to_cache_.end()) {
        const TimeCache& cache = cache_it->second;
        if (cache.parent_frame != parent_frame) {
            return false;
        }
        if (cache.is_static != is_static) {
            return false;
        }
        return true;
    }

    return !would_create_cycle_locked(parent_frame, child_frame);
}

bool TfBuffer::would_create_cycle_locked(
    const std::string& parent_frame,
    const std::string& child_frame
) const {
    std::unordered_set<std::string> visited;
    std::string current = parent_frame;

    while (true) {
        if (current == child_frame) {
            return true;
        }

        if (!visited.insert(current).second) {
            return true;
        }

        const auto cache_it = child_to_cache_.find(current);
        if (cache_it == child_to_cache_.end()) {
            return false;
        }

        current = cache_it->second.parent_frame;
    }
}

void TfBuffer::prune_old_transforms_locked(TimeCache& cache) const {
    if (cache.is_static || cache.transforms.empty()) {
        return;
    }

    const uint64_t latest_stamp = cache.transforms.back().stamp_ns;

    while (cache.transforms.size() > 1) {
        const uint64_t oldest_stamp = cache.transforms.front().stamp_ns;
        if (latest_stamp < oldest_stamp || latest_stamp - oldest_stamp <= cache_time_ns_) {
            break;
        }
        cache.transforms.pop_front();
    }
}

Eigen::Isometry3d TfBuffer::interpolate_transform(
    const TransformStamped& before,
    const TransformStamped& after,
    uint64_t stamp_ns
) {
    if (before.stamp_ns == after.stamp_ns) {
        return before.transform;
    }

    const double ratio = static_cast<double>(stamp_ns - before.stamp_ns) /
        static_cast<double>(after.stamp_ns - before.stamp_ns);

    const Eigen::Vector3d translation = before.transform.translation() +
        ratio * (after.transform.translation() - before.transform.translation());

    Eigen::Quaterniond rotation_before(before.transform.linear());
    Eigen::Quaterniond rotation_after(after.transform.linear());
    rotation_before.normalize();
    rotation_after.normalize();

    Eigen::Quaterniond rotation = rotation_before.slerp(ratio, rotation_after);
    rotation.normalize();

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = rotation.toRotationMatrix();
    result.translation() = translation;
    return result;
}

bool TfBuffer::is_valid_frame_id(const std::string& frame) {
    return !frame.empty() && frame.find('\0') == std::string::npos;
}

bool TfBuffer::is_valid_transform(const Eigen::Isometry3d& transform) {
    if (!transform.matrix().allFinite()) {
        return false;
    }

    const double determinant = transform.linear().determinant();
    return std::isfinite(determinant) && std::abs(determinant) > MIN_VALID_DETERMINANT;
}

} // namespace tools
