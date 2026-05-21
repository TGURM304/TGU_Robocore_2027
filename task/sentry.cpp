//
// Created by tgu on 2026/5/20.
//

#include "tools/BS_thread_pool.hpp"
#include "tools/concurrentqueue.hpp"
#include "tools/foxglove_comm.hpp"
#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"
#include "io/hikrobot/hikrobot.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <thread>

#include "tools/time.hpp"

// 配置文件路径
const auto CONFIG_PATH = "../config/sentry.toml";

// 运行状态
std::atomic_bool running = true;

// logger
tools::LoggerConfig cfg{
    .level = tools::LogLevel::Debug, .enable_console = true, .enable_file = false, .file_path = "logs.txt"
};
static constexpr const char *MODULE = "SENTRY_MAIN";

// 相机
struct Frame {
    cv::Mat image;
    uint64_t timestamp_ns;
};

std::atomic<std::shared_ptr<Frame> > frame{nullptr};

// 相机采集线程
void camera_thread() {
    io::Hikrobot camera(CONFIG_PATH);
    while (running) {
        if (!camera.init()) {
            LOG_ERROR(MODULE, "camera init failed");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        if (!camera.start()) {
            LOG_ERROR(MODULE, "camera start failed");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        cv::Mat image{};
        uint64_t timestamp_ns = 0;
        uint32_t error_count = 0;

        while (camera.is_running() && running) {
            if (error_count >= 20) {
                LOG_ERROR(MODULE, "camera disconnected");
                break;
            }

            if (!camera.grab(image, timestamp_ns)) {
                error_count++;
                continue;
            }

            frame.store(std::make_shared<Frame>(Frame{.image = image.clone(), .timestamp_ns = timestamp_ns}));
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }
    LOG_INFO(MODULE, "Camera thread stopped");
}

// foxglove调试线程
void foxglove_thread() {
    tools::FoxGloveComm comm("0.0.0.0", 8765);

    if (!comm.is_ok()) {
        LOG_ERROR(MODULE, "foxglove server init failed");
        return;
    }

    comm.create_image_channel("/raw_image");
    comm.create_float_channel("/test");

    // 两个常驻任务分别负责图像和数据发布，互不阻塞。
    BS::thread_pool foxglove_pool(2);

    foxglove_pool.detach_task([&comm] {
        while (running) {
            auto frame_ = frame.load();

            if (frame_) {
                try {
                    cv::Mat resized;
                    cv::resize(frame_->image, resized, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
                    comm.publish_image("/raw_image", resized, frame_->timestamp_ns, "camera_raw_frame");
                } catch (const std::exception &e) {
                    LOG_ERROR(MODULE, "async image publish failed: {}", e.what());
                } catch (...) {
                    LOG_ERROR(MODULE, "async image publish failed: unknown exception");
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    foxglove_pool.detach_task([&comm] {
        while (running) {
            const uint64_t timestamp_ns = tools::steady_time_ns();
            const float value = static_cast<float>(std::sin(6.283 * static_cast<double>(timestamp_ns) * 1e-9));

            comm.publish_float("/test", value, timestamp_ns);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main() {
    tools::Logger::instance().init(cfg);

    std::jthread camera(camera_thread);
    std::jthread foxglove(foxglove_thread);

    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
