//
// Created by Interweave on 2026/4/14.
//

#ifndef TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
#define TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "opencv2/opencv.hpp"

namespace tools {
    class FoxGloveComm {
    public:
        static constexpr const char *MODULE = "FOXGLOVE_COMM";

        FoxGloveComm(const std::string &host = "0.0.0.0", uint16_t port = 8765);

        ~FoxGloveComm();

        FoxGloveComm(const FoxGloveComm &) = delete;

        FoxGloveComm &operator=(const FoxGloveComm &) = delete;

        FoxGloveComm(FoxGloveComm &&) noexcept;

        FoxGloveComm &operator=(FoxGloveComm &&) noexcept;

        bool is_ok() const;

        const std::string &get_host() const;

        uint16_t get_port() const;

        bool create_image_channel(const std::string &topic);

        bool publish_image(const std::string &topic, const cv::Mat &image, uint64_t timestamp_ns,
                           const std::string &frame_id = "camera");

        bool create_float_channel(const std::string &topic);

        bool publish_float(const std::string &topic, float value, uint64_t timestamp_ns = 0);

        bool create_point_cloud_channel(const std::string &topic);

        bool publish_point_cloud(const std::string &topic, const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                  uint64_t timestamp_ns, const std::string &frame_id = "cloud_map");

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace tools

#endif //TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
