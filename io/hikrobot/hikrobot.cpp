//
// Created by tgu on 2026/4/16.
//

#include "hikrobot.hpp"

#include "arv.h"
#include <iostream>

namespace io {
    HikRobot::HikRobot(std::string device_id)
        : device_id_(std::move(device_id)) {
    }

    HikRobot::~HikRobot() {
        stop();
        release();
    }

    bool HikRobot::print_error_and_clear(
        GError *&error,
        const char *where
    ) {
        if (error != nullptr) {
            std::cerr
                    << where
                    << " failed: "
                    << error->message
                    << std::endl;

            g_clear_error(&error);

            return true;
        }

        return false;
    }

    bool HikRobot::init() {
        if (!init_camera()) {
            return false;
        }

        if (!init_stream()) {
            return false;
        }

        return true;
    }

    bool HikRobot::init_camera() {
        arv_update_device_list();

        auto n_devices = arv_get_n_devices();

        if (n_devices <= 0) {
            std::cerr
                    << "No camera found"
                    << std::endl;

            return false;
        }

        const char *id = nullptr;

        if (device_id_.empty()) {
            id = arv_get_device_id(0);
        } else {
            id = device_id_.c_str();
        }

        std::cout
                << "Using device: "
                << id
                << std::endl;

        GError *error = nullptr;

        camera_ = arv_camera_new(
            id,
            &error
        );

        if (print_error_and_clear(
                error,
                "arv_camera_new"
            ) || camera_ == nullptr) {
            return false;
        }

        arv_camera_set_exposure_time(
            camera_,
            2000.0,
            &error
        );

        print_error_and_clear(
            error,
            "set_exposure"
        );

        return true;
    }

    bool HikRobot::init_stream() {
        GError *error = nullptr;

        stream_ = arv_camera_create_stream(
            camera_,
            nullptr,
            nullptr,
            &error
        );

        if (print_error_and_clear(
                error,
                "create_stream"
            ) || stream_ == nullptr) {
            return false;
        }

        payload_ = arv_camera_get_payload(
            camera_,
            &error
        );

        if (print_error_and_clear(
                error,
                "get_payload"
            ) || payload_ <= 0) {
            return false;
        }

        std::cout
                << "Payload size: "
                << payload_
                << std::endl;

        // buffer pool
        for (int i = 0; i < 10; ++i) {
            arv_stream_push_buffer(
                stream_,
                arv_buffer_new(
                    payload_,
                    nullptr
                )
            );
        }

        return true;
    }

    bool HikRobot::start() {
        if (camera_ == nullptr) {
            return false;
        }

        GError *error = nullptr;

        arv_camera_start_acquisition(
            camera_,
            &error
        );

        if (print_error_and_clear(
            error,
            "start_acquisition"
        )) {
            return false;
        }

        running_ = true;

        std::cout
                << "Camera streaming started"
                << std::endl;

        return true;
    }

    void HikRobot::stop() {
        if (!running_) {
            return;
        }

        running_ = false;

        GError *error = nullptr;

        if (camera_ != nullptr) {
            arv_camera_stop_acquisition(
                camera_,
                &error
            );

            print_error_and_clear(
                error,
                "stop_acquisition"
            );
        }
    }

    bool HikRobot::grab(
        cv::Mat &image,
        uint64_t &timestamp_ns
    ) {
        if (!running_) {
            return false;
        }

        ArvBuffer *buffer =
                arv_stream_timeout_pop_buffer(
                    stream_,
                    200000
                );

        if (buffer == nullptr) {
            return false;
        }

        if (arv_buffer_get_status(buffer)
            != ARV_BUFFER_STATUS_SUCCESS) {
            arv_stream_push_buffer(
                stream_,
                buffer
            );

            return false;
        }

        size_t size = 0;

        const void *data =
                arv_buffer_get_data(
                    buffer,
                    &size
                );

        gint width =
                arv_buffer_get_image_width(
                    buffer
                );

        gint height =
                arv_buffer_get_image_height(
                    buffer
                );

        if (width <= 0 ||
            height <= 0 ||
            data == nullptr) {
            arv_stream_push_buffer(
                stream_,
                buffer
            );

            return false;
        }

        cv::Mat raw(
            height,
            width,
            CV_8UC1,
            const_cast<void *>(data)
        );

        cv::cvtColor(
            raw,
            image,
            cv::COLOR_BayerRG2RGB
        );

        timestamp_ns =
                (uint64_t) g_get_monotonic_time()
                * 1000ULL;

        arv_stream_push_buffer(
            stream_,
            buffer
        );

        return true;
    }

    bool HikRobot::is_running() const {
        return running_;
    }

    void HikRobot::release() {
        if (stream_ != nullptr) {
            g_object_unref(stream_);

            stream_ = nullptr;
        }

        if (camera_ != nullptr) {
            g_object_unref(camera_);

            camera_ = nullptr;
        }
    }
}
