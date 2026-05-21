/**
 * @file aravis.hpp
 * @brief Aravis 工业相机封装接口。
 * @author Interweave
 *
 * 该模块负责通过 Aravis SDK 初始化相机、启动采集、获取图像和释放资源。
 * 当前输出图像由 BayerRG8 转换为 OpenCV `cv::Mat`，时间戳单位统一为纳秒 ns。
 */

#ifndef TGU_ROBOCORE_2027_ARAVIS_HPP
#define TGU_ROBOCORE_2027_ARAVIS_HPP

#pragma once

#include <arv.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <string>

namespace io {
    /**
     * @brief Aravis 工业相机驱动封装类。
     *
     * 封装 Aravis SDK 的设备枚举、相机创建、曝光/增益配置、数据流创建、连续采集和资源释放。
     *
     */
    class Aravis {
    public:
        /**
         * @brief 构造 Aravis 相机对象并读取配置文件。
         * @param cfg_file_path TOML 配置文件路径
         */
        explicit Aravis(
            std::string_view cfg_file_path = ""
        );

        /**
         * @brief 析构相机对象，自动停止采集并释放 Aravis 资源。
         */
        ~Aravis();

    public:
        /**
         * @brief 初始化相机和数据流。
         *
         * 内部依次调用相机初始化和 stream 初始化。初始化成功后仍需调用 `start()` 才会开始采集。
         * @return true 初始化成功；false 初始化失败。
         */
        bool init();

        /**
         * @brief 开始连续采集图像。
         * @return true 启动成功；false 启动失败。
         */
        bool start();

        /**
         * @brief 停止图像采集。
         */
        void stop();

        /**
         * @brief 获取一帧图像。
         *
         * 内部从 Aravis stream 中取出 buffer，并将当前 BayerRG8 原始图像转换为 OpenCV 图像。
         * @param image 输出图像，当前由 BayerRG8 转换得到。
         * @param timestamp_ns 输出帧时间戳，单位为纳秒 ns，当前使用项目统一单调时钟。
         * @return true 成功获取图像；false 获取失败或相机未运行。
         */
        bool grab(
            cv::Mat &image,
            uint64_t &timestamp_ns
        );

        /**
         * @brief 查询相机是否处于采集运行状态。
         * @return true 正在采集；false 未采集。
         */
        bool is_running() const;

        /**
         * @brief 设置曝光时间。
         * @param exposure_us 曝光时间，单位 us。
         */
        void set_exposure(uint32_t exposure_us) const;

        /**
         * @brief 设置相机增益。
         * @param gain 增益值。
         */
        void set_gain(uint8_t gain) const;

    private:
        /**
         * @brief 初始化 Aravis 相机对象并设置曝光、增益等参数。
         * @return true 初始化成功；false 初始化失败。
         */
        bool init_camera();

        /**
         * @brief 创建 Aravis 数据流并预分配 buffer 池。
         * @return true 初始化成功；false 初始化失败。
         */
        bool init_stream();

        /**
         * @brief 释放 stream 和 camera 等 Aravis 资源。
         */
        void release();

        /**
         * @brief 打印并清理 GLib/Aravis 错误对象。
         * @param error GLib 错误指针引用，非空时会输出日志并清理。
         * @param where 错误发生位置说明。
         * @return true 存在错误并已处理；false 没有错误。
         */
        static bool print_error_and_clear(
            GError *&error,
            const char *where
        );

        /** @brief 目标相机设备 ID；为空时默认使用枚举到的第一台设备。 */
        std::string device_id_;

        /** @brief 曝光时间，单位 us。 */
        uint32_t exposure_us_ = 2000;

        /** @brief 相机增益。 */
        uint8_t gain_ = 5;

        /** @brief 配置文件路径。 */
        std::string_view cfg_file_path_;

        /** @brief Aravis 相机对象指针。 */
        ArvCamera *camera_ = nullptr;

        /** @brief Aravis 数据流对象指针。 */
        ArvStream *stream_ = nullptr;

        /** @brief 单帧图像 payload 大小，单位 byte。 */
        unsigned int payload_ = 0;

        /** @brief 相机采集运行状态。 */
        std::atomic<bool> running_{false};

        /** @brief 日志模块名。 */
        static constexpr auto MODULE = "ARAVIS";
    };
}

#endif //TGU_ROBOCORE_2027_ARAVIS_HPP
