#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace app::point_lio {

struct PointLioPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t timestamp_ns = 0;
    Eigen::Isometry3d odom_T_body = Eigen::Isometry3d::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
};

struct PointLioOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t timestamp_ns = 0;
    PointLioPose odometry;
    std::vector<PointLioPose, Eigen::aligned_allocator<PointLioPose>> path;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_registered{new pcl::PointCloud<pcl::PointXYZI>()};
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_registered_body{new pcl::PointCloud<pcl::PointXYZI>()};
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_map{new pcl::PointCloud<pcl::PointXYZI>()};

    bool has_odometry = false;
    bool has_path = false;
    bool has_cloud_registered = false;
    bool has_cloud_registered_body = false;
    bool has_laser_map = false;
};

} // namespace app::point_lio
