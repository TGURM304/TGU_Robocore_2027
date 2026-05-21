//
// Created by tgu on 2026/4/16.
//

#include "aravis.hpp"

#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

#include "arv.h"

namespace io {
    Aravis::Aravis(std::string_view file_path) : cfg_file_path_(file_path) {
        auto config = toml::parse_file(file_path);

        tools::LoggerConfig cfg{
            .level = tools::LogLevel::Debug,
            .enable_console = true,
            .enable_file = false,
            .file_path = "logs.txt"
        };

        device_id_ = config["camera"]["camera_id"].value_or("");

        exposure_ms_ = config["camera"]["exposure_ms"].value_or(4000);

        gain_ = config["camera"]["gain"].value_or(5);
    }

    Aravis::~Aravis() {
        stop();
        release();
    }

    bool Aravis::print_error_and_clear(
        GError *&error,
        const char *where
    ) {
        if (error != nullptr) {
            LOG_ERROR(MODULE, "failed:{}", error->message);
            g_clear_error(&error);
            return true;
        }
        return false;
    }

    bool Aravis::init() {
        if (!init_camera()) {
            return false;
        }

        if (!init_stream()) {
            return false;
        }

        return true;
    }

    bool Aravis::init_camera() {
        arv_update_device_list();

        auto n_devices = arv_get_n_devices();

        if (n_devices <= 0) {
            LOG_ERROR(MODULE, "No camera found");
            return false;
        }

        const char *id = nullptr;

        if (device_id_.empty()) {
            id = arv_get_device_id(0);
        } else {
            id = device_id_.c_str();
        }

        LOG_INFO(MODULE, "Using device: {}", id);

        GError *error = nullptr;

        camera_ = arv_camera_new(id, &error);

        if (print_error_and_clear(error, "arv_camera_new") || camera_ == nullptr) {
            return false;
        }

        arv_camera_set_exposure_time(camera_, exposure_ms_, &error);

        arv_camera_set_gain(camera_, gain_, &error);

        print_error_and_clear(error, "set_exposure");

        print_error_and_clear(error, "set_gain");

        return true;
    }

    bool Aravis::init_stream() {
        GError *error = nullptr;

        stream_ = arv_camera_create_stream(camera_, nullptr, nullptr, &error);

        if (print_error_and_clear(error, "create_stream") || stream_ == nullptr) {
            return false;
        }

        payload_ = arv_camera_get_payload(camera_, &error);

        if (print_error_and_clear(error, "get_payload") || payload_ <= 0) {
            return false;
        }

        LOG_DEBUG(MODULE, "Payload size: {}", payload_);

        // buffer pool
        for (int i = 0; i < 10; ++i) {
            arv_stream_push_buffer(stream_, arv_buffer_new(payload_, nullptr));
        }

        return true;
    }

    bool Aravis::start() {
        if (camera_ == nullptr) {
            return false;
        }

        GError *error = nullptr;

        arv_camera_start_acquisition(camera_, &error);

        if (print_error_and_clear(error, "start_acquisition")) {
            return false;
        }

        running_ = true;

        LOG_INFO(MODULE, "Camera streaming started");

        return true;
    }

    void Aravis::stop() {
        if (!running_) {
            return;
        }

        running_ = false;

        GError *error = nullptr;

        if (camera_ != nullptr) {
            arv_camera_stop_acquisition(camera_,&error);

            print_error_and_clear(error,"stop_acquisition");
        }
    }

    bool Aravis::grab(cv::Mat &image, uint64_t &timestamp_ns) {
        if (!running_) {
            return false;
        }

        ArvBuffer *buffer = arv_stream_timeout_pop_buffer(stream_, 200000);

        if (buffer == nullptr) {
            return false;
        }

        if (arv_buffer_get_status(buffer)
            != ARV_BUFFER_STATUS_SUCCESS) {
            arv_stream_push_buffer(stream_, buffer);
            return false;
        }

        size_t size = 0;

        const void *data = arv_buffer_get_data(buffer, &size);

        gint width = arv_buffer_get_image_width(buffer);

        gint height = arv_buffer_get_image_height(buffer);

        if (width <= 0 || height <= 0 || data == nullptr) {
            arv_stream_push_buffer(stream_, buffer);
            return false;
        }

        cv::Mat raw(height, width,CV_8UC1, const_cast<void *>(data));

        cv::cvtColor(raw, image, cv::COLOR_BayerRG2RGB);

        timestamp_ns = (uint64_t) g_get_monotonic_time() * 1000ULL;

        arv_stream_push_buffer(stream_, buffer);
        return true;
    }

    bool Aravis::is_running() const {
        return running_;
    }

    void Aravis::release() {
        if (stream_ != nullptr) {
            g_object_unref(stream_);
            stream_ = nullptr;
        }

        if (camera_ != nullptr) {
            g_object_unref(camera_);
            camera_ = nullptr;
        }
    }

    void Aravis::set_exposure(const uint32_t exposure_ms) const {
        GError *error = nullptr;
        arv_camera_set_exposure_time(camera_, exposure_ms, &error);

        print_error_and_clear(error, "set_exposure");
    }

    void Aravis::set_gain(const uint8_t gain) const {
        GError *error = nullptr;
        arv_camera_set_gain(camera_, gain, &error);

        print_error_and_clear(error, "set_exposure");
    }
}
