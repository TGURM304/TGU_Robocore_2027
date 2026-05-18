//
// Created by tgu on 2026/4/16.
//

#ifndef TGU_ROBOCORE_2027_HIKROBOT_HPP
#define TGU_ROBOCORE_2027_HIKROBOT_HPP

#pragma once

#include <arv.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <string>

namespace io {
    class HikRobot {
    public:
        explicit HikRobot(
            std::string device_id = ""
        );

        ~HikRobot();

    public:
        bool init();

        bool start();

        void stop();

        bool grab(
            cv::Mat &image,
            uint64_t &timestamp_ns
        );

        bool is_running() const;

    private:
        bool init_camera();

        bool init_stream();

        void release();

        static bool print_error_and_clear(
            GError *&error,
            const char *where
        );

    private:
        std::string device_id_;

        ArvCamera *camera_ = nullptr;

        ArvStream *stream_ = nullptr;

        unsigned int payload_ = 0;

        std::atomic<bool> running_{false};
    };
}

#endif //TGU_ROBOCORE_2027_HIKROBOT_HPP
