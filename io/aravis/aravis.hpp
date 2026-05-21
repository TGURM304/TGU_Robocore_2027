//
// Created by tgu on 2026/4/16.
//

#ifndef TGU_ROBOCORE_2027_ARAVIS_HPP
#define TGU_ROBOCORE_2027_ARAVIS_HPP

#pragma once

#include <arv.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <string>

namespace io {
    class Aravis {
    public:
        explicit Aravis(
            std::string_view cfg_file_path = ""
        );

        ~Aravis();

    public:
        bool init();

        bool start();

        void stop();

        bool grab(
            cv::Mat &image,
            uint64_t &timestamp_ns
        );

        bool is_running() const;

        void set_exposure(uint32_t exposure_ms) const;

        void set_gain(uint8_t gain) const;

    private:
        bool init_camera();

        bool init_stream();

        void release();

        static bool print_error_and_clear(
            GError *&error,
            const char *where
        );

        std::string device_id_;

        uint32_t exposure_ms_ = 2000;

        uint8_t gain_ = 5;

        std::string_view cfg_file_path_;

        ArvCamera *camera_ = nullptr;

        ArvStream *stream_ = nullptr;

        unsigned int payload_ = 0;

        std::atomic<bool> running_{false};

        static constexpr auto MODULE = "ARAVIS";
    };
}

#endif //TGU_ROBOCORE_2027_ARAVIS_HPP
