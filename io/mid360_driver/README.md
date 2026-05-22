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

#include "mid360_driver/mid360_driver.hpp"
#include "tools/foxglove_comm.hpp"

int main() {
    asio::io_context io_context;
    tools::FoxGloveComm foxglove("0.0.0.0", 8765);

    foxglove.create_point_cloud_channel("/livox/lidar");

    io::Mid360Driver driver(
            io_context,
            "../config/test.toml",
            [&foxglove](const asio::ip::address &lidar_ip,
                        const std::vector<io::Point> &points,
                        uint64_t timestamp_ns) {
                pcl::PointCloud<pcl::PointXYZI> cloud;
                cloud.reserve(points.size());

                for (const auto &src_point: points) {
                    pcl::PointXYZI point{};
                    point.x = src_point.x;
                    point.y = src_point.y;
                    point.z = src_point.z;
                    point.intensity = src_point.intensity;
                    cloud.push_back(point);
                }

                foxglove.publish_point_cloud("/livox/lidar", cloud, timestamp_ns, "livox_frame");
            },
            [](const asio::ip::address &lidar_ip, const io::ImuMsg &imu_msg) {
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
- `offset_time`
- `x`
- `y`
- `z`
- `intensity`

驱动内部已经使用独立线程池分发点云和 IMU 回调。外部回调可以直接做 Point-LIO 数据转换、PCL 转换或 Foxglove 发布，不会阻塞 UDP 接收协程。

构造函数最后两个参数可选，用于配置点云回调线程数和 IMU 回调线程数，默认各 1 个线程。点云和 IMU 使用不同线程池，点云处理较慢时不会把 IMU 回调排在后面。
