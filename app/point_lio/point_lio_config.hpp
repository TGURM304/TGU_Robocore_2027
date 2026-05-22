#pragma once

#include <array>
#include <string_view>

#include <Eigen/Core>

namespace app::point_lio {

struct PointLioConfig {
    bool enabled = true;

    int lidar_type = 1;
    int scan_line = 6;
    double blind = 0.5;

    bool con_frame = false;
    int con_frame_num = 1;
    bool cut_frame = false;
    double cut_frame_time_interval = 0.1;
    double time_diff_lidar_to_imu = 0.0;

    bool imu_en = true;
    bool extrinsic_est_en = false;
    bool use_imu_as_input = false;
    bool space_down_sample = true;
    bool publish_odometry_without_downsample = false;

    double imu_time_inte = 0.005;
    double lidar_time_inte = 0.1;
    double satu_acc = 3.0;
    double satu_gyro = 35.0;
    double acc_norm = 1.0;

    double lidar_meas_cov = 0.01;
    double acc_cov_output = 500.0;
    double gyr_cov_output = 1000.0;
    double b_acc_cov = 0.0001;
    double b_gyr_cov = 0.0001;
    double imu_meas_acc_cov = 0.1;
    double imu_meas_omg_cov = 0.1;
    double gyr_cov_input = 0.01;
    double acc_cov_input = 0.1;

    double plane_thr = 0.1;
    double match_s = 81.0;
    double ivox_grid_resolution = 2.0;
    int ivox_nearby_type = 6;
    double filter_size_surf_min = 0.2;
    double filter_size_map_min = 0.4;
    int init_map_size = 10;
    double det_range = 100.0;
    double fov_degree = 180.0;

    Eigen::Vector3d gravity{0.0, 0.0, -9.81};
    Eigen::Vector3d gravity_init{0.0, 0.0, -9.81};
    Eigen::Vector3d extrinsic_t{0.011, 0.02329, -0.04412};
    Eigen::Matrix3d extrinsic_r = Eigen::Matrix3d::Identity();

    bool path_en = true;
    bool scan_publish_en = true;
    bool scan_bodyframe_pub_en = true;
    bool pcd_save_en = false;
    int pcd_save_interval = -1;
    bool runtime_pos_log = false;
};

PointLioConfig load_config(std::string_view cfg_file_path);

} // namespace app::point_lio
