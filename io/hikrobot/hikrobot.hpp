//
// Created by tgu on 2026/5/21.
//

#ifndef TGU_ROBOCORE_2027_HIKROBOT_HPP
#define TGU_ROBOCORE_2027_HIKROBOT_HPP

#pragma once

#include <atomic>
#include <string_view>

#include <opencv2/opencv.hpp>

#include "hikSDK/include/MvCameraControl.h"

namespace io {

    class Hikrobot {
    public:
        explicit Hikrobot(
            std::string_view cfg_file_path = ""
        );

        ~Hikrobot();

        bool init();

        bool start();

        bool grab(
            cv::Mat& image,
            uint64_t& timestamp_ns
        );

        bool is_running() const;

        void close();

    private:
        void* camera_handle_ = nullptr;

        MV_CC_DEVICE_INFO_LIST device_list_{};

        MV_FRAME_OUT frameOut_{};

        MV_CC_PIXEL_CONVERT_PARAM_EX pstCvtParam_{};

        unsigned char* pDataForRGB_ = nullptr;

        std::string device_id_;

        std::string_view cfg_file_path_;

        uint32_t n_ret_ = MV_OK;

        uint32_t width_ = 0;

        uint32_t height_ = 0;

        uint exposure_ms_ = 1;

        float gain_ = 5.0f;

        std::atomic<bool> running_{false};

        static constexpr auto MODULE =
            "HIKROBOT";
    };

} // namespace io

#endif //TGU_ROBOCORE_2027_HIKROBOT_HPP
