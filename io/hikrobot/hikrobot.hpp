/**
 * @file hikrobot.hpp
 * @brief 海康工业相机 SDK 封装接口。
 * @author Interweave
 *
 * 该模块负责海康 USB 工业相机的初始化、启动采集、图像获取和资源释放。
 * 输出图像为 OpenCV 的 BGR8 `cv::Mat`，时间戳单位统一为纳秒 ns。
 */

#ifndef TGU_ROBOCORE_2027_HIKROBOT_HPP
#define TGU_ROBOCORE_2027_HIKROBOT_HPP

#pragma once

#include <atomic>
#include <string_view>

#include <opencv2/opencv.hpp>

#include "hikSDK/include/MvCameraControl.h"

namespace io {

    /**
     * @brief 海康工业相机驱动封装类。
     *
     * 封装海康 MVS SDK 的设备枚举、句柄创建、参数配置、连续采集、像素格式转换和资源释放。
     *
     */
    class Hikrobot {
    public:
        /**
         * @brief 构造海康相机对象并读取配置文件。
         * @param cfg_file_path TOML 配置文件路径
         */
        explicit Hikrobot(
            std::string_view cfg_file_path = ""
        );

        /**
         * @brief 析构相机对象，自动停止采集、释放句柄并反初始化 SDK。
         */
        ~Hikrobot();

        /**
         * @brief 初始化相机设备并配置采集参数。
         *
         * 主要流程包括枚举 USB 相机、匹配序列号、创建句柄、打开设备、设置触发模式、曝光、增益、像素格式等。
         * @return true 初始化成功；false 初始化失败。
         */
        bool init();

        /**
         * @brief 开始连续采集图像。
         * @return true 启动成功；false 启动失败。
         */
        bool start();

        /**
         * @brief 获取一帧图像。
         *
         * 内部从海康 SDK 获取原始帧，并转换为 BGR8 格式的 OpenCV 图像。
         * @param image 输出图像，格式为 BGR8 `cv::Mat`。
         * @param timestamp_ns 输出帧时间戳，单位为纳秒 ns，当前使用项目统一单调时钟。
         * @return true 成功获取图像；false 获取失败或相机未就绪。
         */
        bool grab(
            cv::Mat& image,
            uint64_t& timestamp_ns
        );

        /**
         * @brief 查询相机是否处于采集运行状态。
         * @return true 正在采集；false 未采集。
         */
        bool is_running() const;

        /**
         * @brief 停止采集并释放相机相关资源。
         *
         * 可重复调用；函数内部会检查句柄和缓存是否有效。
         */
        void close();

    private:
        /** @brief 海康 SDK 相机句柄。 */
        void* camera_handle_ = nullptr;

        /** @brief 海康 SDK 枚举到的设备列表。 */
        MV_CC_DEVICE_INFO_LIST device_list_{};

        /** @brief 海康 SDK 输出帧缓存描述。 */
        MV_FRAME_OUT frameOut_{};

        /** @brief 像素格式转换参数，用于将相机原始数据转换为 BGR8。 */
        MV_CC_PIXEL_CONVERT_PARAM_EX pstCvtParam_{};

        /** @brief BGR 图像转换输出缓存。 */
        unsigned char* pDataForRGB_ = nullptr;

        /** @brief 目标相机序列号 / ID。 */
        std::string device_id_;

        /** @brief 配置文件路径。 */
        std::string_view cfg_file_path_;

        /** @brief 海康 SDK 函数返回值缓存。 */
        uint32_t n_ret_ = MV_OK;

        /** @brief 当前图像宽度，单位 pixel。 */
        uint32_t width_ = 0;

        /** @brief 当前图像高度，单位 pixel。 */
        uint32_t height_ = 0;

        /** @brief 曝光时间，单位 us。 */
        float exposure_us_ = 4000.0f;

        /** @brief 相机增益。 */
        float gain_ = 5.0f;

        /** @brief 相机采集运行状态。 */
        std::atomic<bool> running_{false};

        /** @brief 日志模块名。 */
        static constexpr auto MODULE =
            "HIKROBOT";
    };

} // namespace io

#endif //TGU_ROBOCORE_2027_HIKROBOT_HPP
