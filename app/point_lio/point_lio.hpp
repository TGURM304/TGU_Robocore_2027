#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "app/point_lio/point_lio_config.hpp"
#include "app/point_lio/point_lio_types.hpp"
#include "io/mid360_driver/mid360_driver.hpp"
#include "tools/tf.hpp"

namespace app::point_lio {

class PointLio {
public:
    explicit PointLio(std::string_view cfg_file_path = "");
    ~PointLio();

    bool init();
    void close();

    void push_pointcloud(const std::vector<io::Point> &points, uint64_t timestamp_ns);
    void push_imu(const io::ImuMsg &imu_msg);

    bool process_once(tools::TfBuffer *tf_buffer = nullptr);

    [[nodiscard]] bool is_running() const;
    [[nodiscard]] bool is_initialized() const;
    [[nodiscard]] bool has_new_output() const;
    [[nodiscard]] PointLioOutput get_output() const;
    [[nodiscard]] const PointLioConfig &config() const;

private:
    struct CloudPacket {
        std::vector<io::Point> points;
        uint64_t timestamp_ns = 0;
    };

    std::string_view cfg_file_path_;
    PointLioConfig config_;
    std::atomic_bool running_{false};
    std::atomic_bool initialized_{false};
    mutable std::atomic_bool new_output_{false};

    mutable std::mutex mutex_;
    std::deque<CloudPacket> cloud_queue_;
    std::deque<io::ImuMsg> imu_queue_;
    PointLioOutput output_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_accumulated_{new pcl::PointCloud<pcl::PointXYZI>()};
    int pcd_save_index_ = 0;
    int pcd_frame_count_ = 0;
    std::ofstream runtime_log_;

    bool process_packet_without_core(const CloudPacket &packet, tools::TfBuffer *tf_buffer);
    bool process_packet_with_core(const CloudPacket &packet, tools::TfBuffer *tf_buffer);
    void configure_core();
    void save_pcd_if_needed(const PointLioOutput &output);
    void write_runtime_log(const PointLioOutput &output);
};

} // namespace app::point_lio
