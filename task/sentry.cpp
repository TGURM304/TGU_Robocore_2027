//
// Created by tgu on 2026/5/20.
//

#include "tools/BS_thread_pool.hpp"
#include "tools/concurrentqueue.hpp"
#include "tools/foxglove_comm.hpp"
#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"
#include "io/aravis/aravis.hpp"
#include "io/hikrobot/hikrobot.hpp"


#include <opencv2/opencv.hpp>

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

    cv::Mat resized;

    // 帧数设定
    constexpr auto SEND_INTERVAL = std::chrono::milliseconds(10);

    while (running) {
        auto loop_start = std::chrono::steady_clock::now();

        auto frame_ = frame.load();

        if (!frame_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        cv::resize(frame_->image, resized, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

        comm.publish_image("/raw_image", resized, frame_->timestamp_ns, "camera_raw_frame");

        std::this_thread::sleep_until(loop_start + SEND_INTERVAL);
    }
}

int main() {
    tools::Logger::instance().init(cfg);

    std::jthread camera(camera_thread);
    std::jthread foxglove(foxglove_thread);

    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(100));
    }
    return 0;
}
