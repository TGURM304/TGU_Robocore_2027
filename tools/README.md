# ROBOCORE::TOOLS

`tools/` 用于放和具体机器人任务解耦的通用工具、通信封装和第三方头文件依赖。

## 模块总览

| 文件/目录 | 类型 | 作用 |
| --- | --- | --- |
| `crc.hpp/.cpp` | 源码 | CRC8 / CRC16 计算与校验 |
| `logger.hpp/.cpp` | 源码 | 统一日志系统 |
| `tf.hpp/.cpp` | 源码 | 线程安全轻量级 TF 树 |
| `foxglove_comm.hpp/.cpp` | 源码 | Foxglove WebSocket 通信封装 |
| `time.hpp` | 头文件 | 项目统一时间戳工具 |
| `tomlpp.hpp` | 单头文件 | TOML 配置读取依赖 |
| `BS_thread_pool.hpp` | 单头文件 | 线程池依赖 |
| `concurrentqueue.hpp` | 单头文件 | 无锁队列依赖 |
| `foxglove/` | 第三方库 | Foxglove SDK 源码与库文件 |

## CMake 使用

`tools/CMakeLists.txt` 当前会构建一个 `tools` OBJECT 库，包含：

```cmake
add_library(tools OBJECT
        logger.cpp
        crc.cpp
        foxglove_comm.cpp
        tf.cpp
)
```

如果你的目标用到了 `logger`、`crc`、`tf`、`foxglove_comm` 这些有 `.cpp` 实现的模块，应链接 `tools`：

```cmake
target_link_libraries(your_target
        PRIVATE
        tools
)
```

如果只使用 `time.hpp`、`tomlpp.hpp`、`BS_thread_pool.hpp`、`concurrentqueue.hpp` 这类纯头文件工具，则不需要额外增加实现源文件。

## crc

`tools/crc.hpp` 提供 CRC8 / CRC16 的计算和校验函数：

```cpp
#include "tools/crc.hpp"

uint16_t crc = tools::get_crc16(data, len_without_crc);
bool ok = tools::check_crc16(packet, len_with_crc);
```

接口说明：

- `get_crc8(const uint8_t* data, uint16_t len)`：`len` 不包含 CRC8 本身。
- `check_crc8(const uint8_t* data, uint16_t len)`：`len` 包含末尾 CRC8。
- `get_crc16(const uint8_t* data, uint32_t len)`：`len` 不包含 CRC16 本身。
- `check_crc16(const uint8_t* data, uint32_t len)`：`len` 包含末尾 CRC16。

## logger

一个轻量级、无第三方格式化库依赖的 C++ 日志系统，基于 `std::format`，支持：

- 编译期裁剪：`Release` 下移除 `LOG_DEBUG`
- 运行时日志等级控制
- 控制台输出
- 文件输出
- 多线程安全
- 模块化日志来源标识

初始化方式：

```cpp
#include "tools/logger.hpp"

tools::LoggerConfig cfg{
    .level = tools::LogLevel::Debug,
    .enable_console = true,
    .enable_file = false,
    .file_path = "logs.txt"
};

tools::Logger::instance().init(cfg);
```

日志宏：

```cpp
static constexpr const char *MODULE = "TEST";

LOG_INFO(MODULE, "value={}", 42);
LOG_DEBUG(MODULE, "debug={}", 3.14f);
LOG_WARN(MODULE, "warn {}", "message");
LOG_ERROR(MODULE, "error code={}", -1);
```

说明：

- `module` 建议使用每个文件自己的静态字符串。
- `fmt` 使用 `std::format` 语法。
- `LOG_DEBUG` 在 `NDEBUG` 下会被编译期移除。
- 示例见 `task/test/test_logger.cpp`。

## time

`tools/time.hpp` 提供项目统一时间戳函数：

```cpp
#include "tools/time.hpp"

uint64_t frame_timestamp_ns = tools::steady_time_ns();
uint64_t unix_timestamp_ns = tools::system_time_ns();
```

接口说明：

- `tools::steady_time_ns()`：单调时钟，单位 ns，适合帧时间戳、延迟统计、预测等内部时间计算。
- `tools::system_time_ns()`：系统 Unix 时间戳，单位 ns，适合日志、录包、外部系统时间对齐。

## tf

`tools/tf.hpp` / `tools/tf.cpp` 提供一个线程安全的轻量级 TF 树，使用语义接近 ROS2 `tf2`：

- 支持静态变换和动态变换
- 支持按时间查询 `target_T_source`
- 支持点、向量、位姿变换
- 查询失败时抛出清晰的异常类型

基本使用：

```cpp
#include "tools/tf.hpp"

tools::TfBuffer tf;

Eigen::Isometry3d base_T_lidar = Eigen::Isometry3d::Identity();
tf.set_static_transform("base_link", "lidar", base_T_lidar);

Eigen::Isometry3d map_T_base = Eigen::Isometry3d::Identity();
tf.set_transform("map", "base_link", map_T_base, tools::system_time_ns());

Eigen::Isometry3d map_T_lidar = tf.lookup_transform("map", "lidar");
```

主要接口：

- `set_transform(...)`
- `set_static_transform(...)`
- `lookup_transform(...)`
- `can_transform(...)`
- `transform_point(...)`
- `transform_vector(...)`
- `transform_pose(...)`
- `all_frames_as_string()`

异常类型：

- `tools::TfException`
- `tools::FrameNotFoundException`
- `tools::ConnectivityException`
- `tools::ExtrapolationException`

## foxglove_comm

`tools::FoxGloveComm` 是对 Foxglove WebSocket Server 的轻量封装，当前支持：

- JPEG 图像发布
- 单个浮点值发布
- PCL 点云发布

当前点云接口使用：

```cpp
const pcl::PointCloud<pcl::PointXYZI> &cloud
```

发布到 Foxglove 时字段为：

- `x`
- `y`
- `z`
- `intensity`

基本使用：

```cpp
#include "tools/foxglove_comm.hpp"
#include "tools/time.hpp"

tools::FoxGloveComm comm("0.0.0.0", 8765);

if (!comm.is_ok()) {
    return -1;
}

comm.create_point_cloud_channel("/lidar");

pcl::PointCloud<pcl::PointXYZI> cloud;
pcl::PointXYZI point;
point.x = 1.0f;
point.y = 2.0f;
point.z = 3.0f;
point.intensity = 120.0f;
cloud.push_back(point);

comm.publish_point_cloud("/lidar", cloud, tools::system_time_ns(), "lidar_frame");
```

注意事项：

- 发布前必须先调用对应的 `create_*_channel()`。
- `publish_image()` 输入是 `cv::Mat`，内部会编码成 JPEG。
- `publish_float()` 走 JSON schema 通道。
- `publish_point_cloud()` 会跳过非有限值点。
- 示例入口见 `task/test/test_foxglove.cpp`。

## tomlpp

`tools/tomlpp.hpp` 是当前项目直接使用的 TOML 单头文件依赖。

简单示例：

```cpp
#include "tools/tomlpp.hpp"

auto config = toml::parse_file("../../config/test.toml");
std::string title = config["title"].value_or("default");
```

更完整示例见 `task/test/read_toml.cpp`。

## 其它头文件依赖

这些文件当前作为可直接包含的第三方/公共头文件保留在 `tools/`：

- `BS_thread_pool.hpp`
- `concurrentqueue.hpp`

README 目前不额外展开它们的 API；如果后续项目中开始正式使用，建议再补独立示例。

## 依赖说明

`tools` 模块当前的主要编译期依赖包括：

- C++20
- OpenCV
- PCL
- Eigen3
- Foxglove SDK

其中：

- `logger` 依赖 `std::format`
- `tf` 依赖 Eigen
- `foxglove_comm` 依赖 OpenCV、PCL 和 `foxglove_sdk`
- `crc`、`time` 为纯轻量工具，无额外重依赖
