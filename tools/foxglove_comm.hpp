//
// Created by Interweave on 2026/4/14.
//

#ifndef TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
#define TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
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

        bool create_pose_channel(const std::string &topic);

        bool publish_pose(const std::string &topic, const Eigen::Isometry3d &pose, uint64_t timestamp_ns,
                          const std::string &frame_id = "odom");

        bool create_path_channel(const std::string &topic);

        bool publish_path(const std::string &topic, const std::vector<Eigen::Isometry3d> &poses,
                          uint64_t timestamp_ns, const std::string &frame_id = "odom");

        bool create_transform_channel(const std::string &topic);

        bool publish_transform(const std::string &topic, const Eigen::Isometry3d &parent_T_child,
                               uint64_t timestamp_ns, const std::string &parent_frame_id,
                               const std::string &child_frame_id);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace tools

#endif //TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
