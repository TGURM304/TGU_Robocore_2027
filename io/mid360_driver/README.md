# mid360_driver

纯 C++ MID-360 驱动核心库，提取自 `Yancey2023/mid360_driver`。

该目录只保留 UDP 通信和数据解析逻辑，不包含 ROS 2 节点、launch、参数文件或 `sensor_msgs` 转换代码。

## 依赖

优先使用 standalone Asio：

```bash
sudo apt install libasio-dev
```

如果系统没有 `asio.hpp`，CMake 会自动回退到 Boost.Asio。本仓库顶层已经依赖 Boost。

## CMake 使用

本仓库已经把 MID-360 驱动源码直接编入 `io` 静态库。需要使用 MID-360 时，目标链接 `io` 即可。
如果要直接发布到 Foxglove，再同时链接 `tools`：

```cmake
target_link_libraries(your_target
        PRIVATE
        io
        tools
)
```

## 代码使用

```cpp
#include <iostream>

#include "tools/BS_thread_pool.hpp"
#include "tools/concurrentqueue.hpp"
#include "mid360_driver/mid360_driver.hpp"
#include "tools/foxglove_comm.hpp"
#include "tools/time.hpp"

namespace {
    struct PointCloudTask {
        std::string lidar_ip;
        std::shared_ptr<std::vector<io::Point>> points;
        uint64_t timestamp_ns = 0;
    };
}

int main() {
    asio::io_context io_context;
    tools::FoxGloveComm foxglove("0.0.0.0", 8765);
    BS::thread_pool publish_pool(1);
    std::atomic_uint64_t last_imu_log_ns = 0;
    moodycamel::ConcurrentQueue<PointCloudTask> point_cloud_queue;

    foxglove.create_point_cloud_channel("/livox/lidar");

    publish_pool.detach_task([&foxglove, &point_cloud_queue] {
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

            std::cout << "pointcloud from " << task.lidar_ip
                      << ", points=" << cloud.size() << std::endl;
            foxglove.publish_point_cloud("/livox/lidar", cloud, task.timestamp_ns, "livox_frame");
        }
    });

    io::Mid360Driver driver(
            io_context,
            "../config/test.toml",
            [&point_cloud_queue](const asio::ip::address &lidar_ip,
                                 const std::vector<io::Point> &points,
                                 uint64_t timestamp_ns) {
                point_cloud_queue.enqueue(PointCloudTask{
                        .lidar_ip = lidar_ip.to_string(),
                        .points = std::make_shared<std::vector<io::Point>>(points),
                        .timestamp_ns = timestamp_ns,
                });
            },
            [&last_imu_log_ns](const asio::ip::address &lidar_ip, const io::ImuMsg &imu_msg) {
                const uint64_t now_ns = tools::steady_time_ns();
                const uint64_t previous_ns = last_imu_log_ns.load(std::memory_order_relaxed);
                if (previous_ns != 0 && now_ns - previous_ns < 100'000'000ULL) {
                    return;
                }

                last_imu_log_ns.store(now_ns, std::memory_order_relaxed);
                std::cout << "imu from " << lidar_ip.to_string()
                           << ", gyro_z=" << imu_msg.angular_velocity_z << std::endl;
            });

    io_context.run();
    return 0;
}
```

需要在 TOML 中配置本机连接 MID-360 的网卡 IP：

```toml
[mid360_driver]
host_ip = "<your_nic_ip>"
```

库内部监听点云端口 `56301` 和 IMU 端口 `56401`，并接收来自雷达 `56300`、`56400` 端口的数据。

点云回调输出类型为 `const std::vector<io::Point>&`，其中每个点包含：

- `timestamp`
- `x`
- `y`
- `z`
- `intensity`

如果需要发布到 Foxglove，建议像 `task/test/test_mid360.cpp` 一样：

- IO 回调线程只负责把 `std::vector<io::Point>` 入队
- `BS::thread_pool` 消费队列并完成 PCL 转换与 Foxglove 发布

这样可以避免点云打包和网络发送阻塞 IMU 回调。
