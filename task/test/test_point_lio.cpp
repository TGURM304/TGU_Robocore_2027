#include "app/point_lio/point_lio.hpp"
#include "io/mid360_driver/mid360_driver.hpp"
#include "tools/foxglove_comm.hpp"
#include "tools/logger.hpp"
#include "tools/tf.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

namespace {

constexpr const char *CONFIG_PATH = "../config/sentry.toml";
constexpr const char *MODULE = "TEST_POINT_LIO";

std::vector<Eigen::Isometry3d> make_path_poses(
        const std::vector<app::point_lio::PointLioPose, Eigen::aligned_allocator<app::point_lio::PointLioPose>> &path) {
    std::vector<Eigen::Isometry3d> poses;
    poses.reserve(path.size());
    for (const auto &pose: path) {
        poses.emplace_back(pose.odom_T_body);
    }
    return poses;
}

} // namespace

int main() {
    tools::Logger::instance().init({.level = tools::LogLevel::Debug, .enable_console = true, .enable_file = false});

    tools::FoxGloveComm comm("0.0.0.0", 8765);
    if (!comm.is_ok()) {
        std::cerr << "foxglove init failed\n";
        return -1;
    }

    comm.create_point_cloud_channel("/point_lio/cloud_registered");
    comm.create_point_cloud_channel("/point_lio/cloud_registered_body");
    comm.create_point_cloud_channel("/point_lio/laser_map");
    comm.create_pose_channel("/point_lio/odometry");
    comm.create_path_channel("/point_lio/path");
    comm.create_transform_channel("/tf");

    tools::TfBuffer tf;
    app::point_lio::PointLio point_lio(CONFIG_PATH);
    if (!point_lio.init()) {
        LOG_ERROR(MODULE, "point_lio init failed");
        return -1;
    }

    asio::io_context io_context;
    io::Mid360Driver driver(
            io_context,
            CONFIG_PATH,
            [&point_lio](const asio::ip::address &, const std::vector<io::Point> &points, uint64_t timestamp_ns) {
                point_lio.push_pointcloud(points, timestamp_ns);
            },
            [&point_lio](const asio::ip::address &, const io::ImuMsg &imu_msg) {
                point_lio.push_imu(imu_msg);
            });

    std::atomic_bool running = true;
    std::jthread asio_thread([&] {
        io_context.run();
        running.store(false, std::memory_order_relaxed);
    });

    while (running.load(std::memory_order_relaxed)) {
        if (!point_lio.process_once(&tf)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const auto output = point_lio.get_output();
        if (output.has_odometry) {
            comm.publish_pose("/point_lio/odometry", output.odometry.odom_T_body, output.timestamp_ns, "odom");
            comm.publish_transform("/tf", output.odometry.odom_T_body, output.timestamp_ns, "odom", "base_link");
        }
        if (output.has_path) {
            comm.publish_path("/point_lio/path", make_path_poses(output.path), output.timestamp_ns, "odom");
        }
        if (output.has_cloud_registered) {
            comm.publish_point_cloud("/point_lio/cloud_registered", *output.cloud_registered, output.timestamp_ns, "odom");
        }
        if (output.has_cloud_registered_body) {
            comm.publish_point_cloud("/point_lio/cloud_registered_body", *output.cloud_registered_body,
                                     output.timestamp_ns, "base_link");
        }
        if (output.has_laser_map) {
            comm.publish_point_cloud("/point_lio/laser_map", *output.laser_map, output.timestamp_ns, "odom");
        }
    }

    return 0;
}
