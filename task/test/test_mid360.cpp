#include "io/mid360_driver/mid360_driver.hpp"
#include "tools/BS_thread_pool.hpp"
#include "tools/concurrentqueue.hpp"
#include "tools/foxglove_comm.hpp"
#include "tools/time.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <iostream>

namespace {
    struct PointCloudTask {
        std::string lidar_ip;
        std::shared_ptr<std::vector<io::Point> > points;
        uint64_t timestamp_ns = 0;
    };
}

int main() {
    tools::FoxGloveComm comm("0.0.0.0", 8765);
    if (!comm.is_ok()) {
        std::cerr << "foxglove init failed\n";
        return -1;
    }

    comm.create_point_cloud_channel("/livox/lidar");

    asio::io_context io_context;
    BS::thread_pool publish_pool(1);
    std::atomic_uint64_t last_imu_log_ns = 0;
    moodycamel::ConcurrentQueue<PointCloudTask> point_cloud_queue;

    publish_pool.detach_task([&comm, &point_cloud_queue] {
        PointCloudTask task;
        while (true) {
            if (!point_cloud_queue.try_dequeue(task)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            pcl::PointCloud<pcl::PointXYZI> cloud;
            cloud.reserve(task.points->size());

            for (const auto &src_point: *task.points) {
                pcl::PointXYZI point{};
                point.x = src_point.x;
                point.y = src_point.y;
                point.z = src_point.z;
                point.intensity = src_point.intensity;
                cloud.push_back(point);
            }

            // std::cout << "pointcloud from " << task.lidar_ip
            //           << ", points=" << cloud.size() << std::endl;
            comm.publish_point_cloud("/livox/lidar", cloud, task.timestamp_ns, "livox_frame");
        }
    });

    io::Mid360Driver driver(io_context, "../config/test.toml",
                            [&point_cloud_queue](const asio::ip::address &lidar_ip,
                                                 const std::vector<io::Point> &points, uint64_t timestamp_ns) {
                                point_cloud_queue.enqueue(PointCloudTask{
                                    .lidar_ip = lidar_ip.to_string(),
                                    .points = std::make_shared<std::vector<io::Point> >(points),
                                    .timestamp_ns = timestamp_ns,
                                });
                            }, [&last_imu_log_ns](const asio::ip::address &lidar_ip, const io::ImuMsg &imu_msg) {
                                const uint64_t now_ns = tools::steady_time_ns();
                                const uint64_t previous_ns = last_imu_log_ns.load(std::memory_order_relaxed);
                                // if (previous_ns != 0 && now_ns - previous_ns < 100'000'000ULL) {
                                //     return;
                                // }

                                last_imu_log_ns.store(now_ns, std::memory_order_relaxed);
                                std::cout << "imu from " << lidar_ip.to_string() << ", gyro_z=" << imu_msg.
                                        angular_velocity_z << ", acc_z=" << imu_msg.linear_acceleration_z << std::endl;
                            });

    io_context.run();
    return 0;
}
