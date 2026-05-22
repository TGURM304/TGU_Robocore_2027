#include "point_lio_config.hpp"

#include <cstddef>
#include <string>

#include "tools/tomlpp.hpp"

namespace app::point_lio {
namespace {

Eigen::Vector3d read_vec3(const toml::table &config, const char *name, const Eigen::Vector3d &fallback) {
    const auto arr = config["point_lio"][name].as_array();
    if (arr == nullptr || arr->size() < 3) {
        return fallback;
    }

    return Eigen::Vector3d((*arr)[0].value_or(fallback.x()),
                           (*arr)[1].value_or(fallback.y()),
                           (*arr)[2].value_or(fallback.z()));
}

Eigen::Matrix3d read_mat3(const toml::table &config, const char *name, const Eigen::Matrix3d &fallback) {
    const auto arr = config["point_lio"][name].as_array();
    if (arr == nullptr || arr->size() < 9) {
        return fallback;
    }

    Eigen::Matrix3d result;
    for (std::size_t i = 0; i < 9; ++i) {
        result(static_cast<Eigen::Index>(i / 3), static_cast<Eigen::Index>(i % 3)) =
                (*arr)[i].value_or(fallback(static_cast<Eigen::Index>(i / 3), static_cast<Eigen::Index>(i % 3)));
    }
    return result;
}

} // namespace

PointLioConfig load_config(std::string_view cfg_file_path) {
    PointLioConfig cfg;
    if (cfg_file_path.empty()) {
        return cfg;
    }

    auto config = toml::parse_file(std::string(cfg_file_path));
    const auto node = config["point_lio"];

    cfg.enabled = node["enabled"].value_or(cfg.enabled);
    cfg.lidar_type = node["lidar_type"].value_or(cfg.lidar_type);
    cfg.scan_line = node["scan_line"].value_or(cfg.scan_line);
    cfg.blind = node["blind"].value_or(cfg.blind);
    cfg.con_frame = node["con_frame"].value_or(cfg.con_frame);
    cfg.con_frame_num = node["con_frame_num"].value_or(cfg.con_frame_num);
    cfg.cut_frame = node["cut_frame"].value_or(cfg.cut_frame);
    cfg.cut_frame_time_interval = node["cut_frame_time_interval"].value_or(cfg.cut_frame_time_interval);
    cfg.time_diff_lidar_to_imu = node["time_diff_lidar_to_imu"].value_or(cfg.time_diff_lidar_to_imu);
    cfg.imu_en = node["imu_en"].value_or(cfg.imu_en);
    cfg.extrinsic_est_en = node["extrinsic_est_en"].value_or(cfg.extrinsic_est_en);
    cfg.use_imu_as_input = node["use_imu_as_input"].value_or(cfg.use_imu_as_input);
    cfg.space_down_sample = node["space_down_sample"].value_or(cfg.space_down_sample);
    cfg.publish_odometry_without_downsample = node["publish_odometry_without_downsample"].value_or(cfg.publish_odometry_without_downsample);
    cfg.imu_time_inte = node["imu_time_inte"].value_or(cfg.imu_time_inte);
    cfg.lidar_time_inte = node["lidar_time_inte"].value_or(cfg.lidar_time_inte);
    cfg.satu_acc = node["satu_acc"].value_or(cfg.satu_acc);
    cfg.satu_gyro = node["satu_gyro"].value_or(cfg.satu_gyro);
    cfg.acc_norm = node["acc_norm"].value_or(cfg.acc_norm);
    cfg.lidar_meas_cov = node["lidar_meas_cov"].value_or(cfg.lidar_meas_cov);
    cfg.acc_cov_output = node["acc_cov_output"].value_or(cfg.acc_cov_output);
    cfg.gyr_cov_output = node["gyr_cov_output"].value_or(cfg.gyr_cov_output);
    cfg.b_acc_cov = node["b_acc_cov"].value_or(cfg.b_acc_cov);
    cfg.b_gyr_cov = node["b_gyr_cov"].value_or(cfg.b_gyr_cov);
    cfg.imu_meas_acc_cov = node["imu_meas_acc_cov"].value_or(cfg.imu_meas_acc_cov);
    cfg.imu_meas_omg_cov = node["imu_meas_omg_cov"].value_or(cfg.imu_meas_omg_cov);
    cfg.gyr_cov_input = node["gyr_cov_input"].value_or(cfg.gyr_cov_input);
    cfg.acc_cov_input = node["acc_cov_input"].value_or(cfg.acc_cov_input);
    cfg.plane_thr = node["plane_thr"].value_or(cfg.plane_thr);
    cfg.match_s = node["match_s"].value_or(cfg.match_s);
    cfg.ivox_grid_resolution = node["ivox_grid_resolution"].value_or(cfg.ivox_grid_resolution);
    cfg.ivox_nearby_type = node["ivox_nearby_type"].value_or(cfg.ivox_nearby_type);
    cfg.filter_size_surf_min = node["filter_size_surf_min"].value_or(cfg.filter_size_surf_min);
    cfg.filter_size_map_min = node["filter_size_map_min"].value_or(cfg.filter_size_map_min);
    cfg.init_map_size = node["init_map_size"].value_or(cfg.init_map_size);
    cfg.det_range = node["det_range"].value_or(cfg.det_range);
    cfg.fov_degree = node["fov_degree"].value_or(cfg.fov_degree);
    cfg.path_en = node["path_en"].value_or(cfg.path_en);
    cfg.scan_publish_en = node["scan_publish_en"].value_or(cfg.scan_publish_en);
    cfg.scan_bodyframe_pub_en = node["scan_bodyframe_pub_en"].value_or(cfg.scan_bodyframe_pub_en);
    cfg.pcd_save_en = node["pcd_save_en"].value_or(cfg.pcd_save_en);
    cfg.pcd_save_interval = node["pcd_save_interval"].value_or(cfg.pcd_save_interval);
    cfg.runtime_pos_log = node["runtime_pos_log"].value_or(cfg.runtime_pos_log);
    cfg.gravity = read_vec3(config, "gravity", cfg.gravity);
    cfg.gravity_init = read_vec3(config, "gravity_init", cfg.gravity_init);
    cfg.extrinsic_t = read_vec3(config, "extrinsic_t", cfg.extrinsic_t);
    cfg.extrinsic_r = read_mat3(config, "extrinsic_r", cfg.extrinsic_r);

    return cfg;
}

} // namespace app::point_lio
