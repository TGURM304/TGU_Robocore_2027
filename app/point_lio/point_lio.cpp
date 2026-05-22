#include "point_lio.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <utility>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "tools/logger.hpp"

#include "third_party/point_lio/src/Estimator.h"
#include "third_party/point_lio/src/IMU_Processing.h"
#include "third_party/point_lio/src/parameters.h"

namespace app::point_lio {
namespace {

constexpr const char *MODULE = "POINT_LIO";
constexpr const char *ODOM_FRAME = "odom";
constexpr const char *BODY_FRAME = "base_link";
constexpr const char *PCD_DIR = "point_lio_pcd";
constexpr const char *LOG_DIR = "point_lio_log";

pcl::PointXYZI to_pcl_point(const io::Point &src) {
    pcl::PointXYZI dst{};
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = src.intensity;
    return dst;
}

PointType to_point_lio_point(const io::Point &src) {
    PointType dst{};
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = src.intensity;
    dst.curvature = static_cast<float>(src.offset_time * 1000.0);
    return dst;
}

pcl::PointXYZI to_xyzi_point(const PointType &src) {
    pcl::PointXYZI dst{};
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = src.intensity;
    return dst;
}

std::shared_ptr<sensor_msgs::Imu> to_point_lio_imu(const io::ImuMsg &src) {
    auto dst = std::make_shared<sensor_msgs::Imu>();
    dst->header.stamp = ros::Time(src.timestamp);
    dst->angular_velocity.x = src.angular_velocity_x;
    dst->angular_velocity.y = src.angular_velocity_y;
    dst->angular_velocity.z = src.angular_velocity_z;
    dst->linear_acceleration.x = src.linear_acceleration_x;
    dst->linear_acceleration.y = src.linear_acceleration_y;
    dst->linear_acceleration.z = src.linear_acceleration_z;
    return dst;
}

Eigen::Isometry3d make_pose_from_core() {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (use_imu_as_input) {
        pose.linear() = kf_input.x_.rot;
        pose.translation() = kf_input.x_.pos;
    } else {
        pose.linear() = kf_output.x_.rot;
        pose.translation() = kf_output.x_.pos;
    }
    return pose;
}

} // namespace

PointLio::PointLio(std::string_view cfg_file_path)
    : cfg_file_path_(cfg_file_path), config_(load_config(cfg_file_path)) {}

PointLio::~PointLio() {
    close();
}

bool PointLio::init() {
    if (!config_.enabled) {
        LOG_WARN(MODULE, "point_lio disabled by config");
        return false;
    }

    running_.store(true, std::memory_order_relaxed);
    initialized_.store(true, std::memory_order_relaxed);
    configure_core();
    if (config_.runtime_pos_log) {
        std::filesystem::create_directories(LOG_DIR);
        runtime_log_.open(std::filesystem::path(LOG_DIR) / "odom_log.txt", std::ios::out);
    }
    LOG_INFO(MODULE, "Point-LIO ROS-free shell initialized with odom frame output");
    return true;
}

void PointLio::close() {
    if (config_.pcd_save_en && pcd_accumulated_ && !pcd_accumulated_->empty()) {
        std::filesystem::create_directories(PCD_DIR);
        const auto file = std::filesystem::path(PCD_DIR) / "scans_final.pcd";
        pcl::io::savePCDFileBinary(file.string(), *pcd_accumulated_);
        pcd_accumulated_->clear();
    }
    if (runtime_log_.is_open()) {
        runtime_log_.close();
    }
    running_.store(false, std::memory_order_relaxed);
    initialized_.store(false, std::memory_order_relaxed);
}

void PointLio::push_pointcloud(const std::vector<io::Point> &points, uint64_t timestamp_ns) {
    if (!running_.load(std::memory_order_relaxed)) {
        return;
    }

    std::lock_guard lock(mutex_);
    cloud_queue_.push_back(CloudPacket{.points = points, .timestamp_ns = timestamp_ns});
    while (cloud_queue_.size() > 8) {
        cloud_queue_.pop_front();
    }
}

void PointLio::push_imu(const io::ImuMsg &imu_msg) {
    if (!running_.load(std::memory_order_relaxed)) {
        return;
    }

    std::lock_guard lock(mutex_);
    imu_queue_.push_back(imu_msg);
    while (imu_queue_.size() > 4000) {
        imu_queue_.pop_front();
    }
}

bool PointLio::process_once(tools::TfBuffer *tf_buffer) {
    if (!running_.load(std::memory_order_relaxed)) {
        return false;
    }

    CloudPacket packet;
    {
        std::lock_guard lock(mutex_);
        if (cloud_queue_.empty()) {
            return false;
        }
        packet = std::move(cloud_queue_.front());
        cloud_queue_.pop_front();

        const double packet_time = static_cast<double>(packet.timestamp_ns) * 1e-9;
        while (!imu_queue_.empty() && imu_queue_.front().timestamp < packet_time - 1.0) {
            imu_queue_.pop_front();
        }
    }

    return process_packet_with_core(packet, tf_buffer);
}

bool PointLio::is_running() const {
    return running_.load(std::memory_order_relaxed);
}

bool PointLio::is_initialized() const {
    return initialized_.load(std::memory_order_relaxed);
}

bool PointLio::has_new_output() const {
    return new_output_.load(std::memory_order_relaxed);
}

PointLioOutput PointLio::get_output() const {
    std::lock_guard lock(mutex_);
    new_output_.store(false, std::memory_order_relaxed);
    return output_;
}

const PointLioConfig &PointLio::config() const {
    return config_;
}

bool PointLio::process_packet_without_core(const CloudPacket &packet, tools::TfBuffer *tf_buffer) {
    // Temporary adapter output: keeps the project IO/TF/Foxglove path live while the staged
    // Point-LIO third_party core is being de-ROSed into this class.
    PointLioOutput next;
    next.timestamp_ns = packet.timestamp_ns;
    next.odometry.timestamp_ns = packet.timestamp_ns;
    next.odometry.gravity = config_.gravity;
    next.has_odometry = true;
    next.has_path = config_.path_en;
    next.has_cloud_registered = config_.scan_publish_en;
    next.has_cloud_registered_body = config_.scan_publish_en && config_.scan_bodyframe_pub_en;
    next.has_laser_map = config_.scan_publish_en;

    next.cloud_registered->reserve(packet.points.size());
    next.cloud_registered_body->reserve(packet.points.size());
    next.laser_map->reserve(packet.points.size());

    for (const auto &point: packet.points) {
        const auto pcl_point = to_pcl_point(point);
        next.cloud_registered->push_back(pcl_point);
        next.cloud_registered_body->push_back(pcl_point);
        next.laser_map->push_back(pcl_point);
    }

    {
        std::lock_guard lock(mutex_);
        next.path = output_.path;
        if (config_.path_en) {
            next.path.push_back(next.odometry);
            if (next.path.size() > 5000) {
                next.path.erase(next.path.begin(), next.path.begin() + static_cast<long>(next.path.size() - 5000));
            }
        }
        output_ = std::move(next);
    }

    if (tf_buffer != nullptr) {
        tf_buffer->set_transform(ODOM_FRAME, BODY_FRAME, Eigen::Isometry3d::Identity(), packet.timestamp_ns);
    }

    {
        std::lock_guard lock(mutex_);
        save_pcd_if_needed(output_);
        write_runtime_log(output_);
    }

    new_output_.store(true, std::memory_order_relaxed);
    return true;
}

bool PointLio::process_packet_with_core(const CloudPacket &packet, tools::TfBuffer *tf_buffer) {
    MeasureGroup measure;
    measure.lidar_beg_time = static_cast<double>(packet.timestamp_ns) * 1e-9;
    measure.lidar_last_time = measure.lidar_beg_time + config_.lidar_time_inte;
    measure.lidar.reset(new PointCloudXYZI());
    measure.lidar->reserve(packet.points.size());

    const double min_range2 = config_.blind * config_.blind;
    const double max_range2 = config_.det_range * config_.det_range;
    for (const auto &src: packet.points) {
        const double range2 = static_cast<double>(src.x) * src.x + static_cast<double>(src.y) * src.y + static_cast<double>(src.z) * src.z;
        if (range2 < min_range2 || range2 > max_range2) {
            continue;
        }
        measure.lidar->push_back(to_point_lio_point(src));
    }
    std::sort(measure.lidar->points.begin(), measure.lidar->points.end(), time_list);

    {
        std::lock_guard lock(mutex_);
        const double end_time = measure.lidar_last_time;
        while (!imu_queue_.empty() && imu_queue_.front().timestamp <= end_time) {
            measure.imu.emplace_back(to_point_lio_imu(imu_queue_.front()));
            imu_queue_.pop_front();
        }
    }

    PointCloudXYZI::Ptr undistorted(new PointCloudXYZI());
    if (p_imu) {
        p_imu->Process(measure, undistorted);
    }
    if (undistorted->empty()) {
        *undistorted = *measure.lidar;
    }

    PointCloudXYZI::Ptr down_body(new PointCloudXYZI());
    if (config_.space_down_sample && !undistorted->empty()) {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(static_cast<float>(config_.filter_size_surf_min),
                          static_cast<float>(config_.filter_size_surf_min),
                          static_cast<float>(config_.filter_size_surf_min));
        voxel.setInputCloud(undistorted);
        voxel.filter(*down_body);
    } else {
        *down_body = *undistorted;
    }

    PointLioOutput next;
    next.timestamp_ns = packet.timestamp_ns;
    next.odometry.timestamp_ns = packet.timestamp_ns;
    next.odometry.odom_T_body = make_pose_from_core();
    next.odometry.gravity = config_.gravity;
    if (use_imu_as_input) {
        next.odometry.velocity = kf_input.x_.vel;
        next.odometry.gyro_bias = kf_input.x_.bg;
        next.odometry.acc_bias = kf_input.x_.ba;
    } else {
        next.odometry.velocity = kf_output.x_.vel;
        next.odometry.gyro_bias = kf_output.x_.bg;
        next.odometry.acc_bias = kf_output.x_.ba;
    }
    next.has_odometry = true;
    next.has_path = config_.path_en;
    next.has_cloud_registered = config_.scan_publish_en;
    next.has_cloud_registered_body = config_.scan_publish_en && config_.scan_bodyframe_pub_en;
    next.has_laser_map = config_.scan_publish_en;

    next.cloud_registered->reserve(down_body->size());
    next.cloud_registered_body->reserve(down_body->size());
    next.laser_map->reserve(down_body->size());
    const Eigen::Isometry3d odom_T_body = next.odometry.odom_T_body;
    for (const auto &point: down_body->points) {
        next.cloud_registered_body->push_back(to_xyzi_point(point));

        const Eigen::Vector3d p_body(point.x, point.y, point.z);
        const Eigen::Vector3d p_odom = odom_T_body * p_body;
        pcl::PointXYZI odom_point{};
        odom_point.x = static_cast<float>(p_odom.x());
        odom_point.y = static_cast<float>(p_odom.y());
        odom_point.z = static_cast<float>(p_odom.z());
        odom_point.intensity = point.intensity;
        next.cloud_registered->push_back(odom_point);
        next.laser_map->push_back(odom_point);
    }

    Eigen::Isometry3d odom_T_body_for_tf = next.odometry.odom_T_body;
    {
        std::lock_guard lock(mutex_);
        next.path = output_.path;
        if (config_.path_en) {
            next.path.push_back(next.odometry);
            if (next.path.size() > 5000) {
                next.path.erase(next.path.begin(), next.path.begin() + static_cast<long>(next.path.size() - 5000));
            }
        }
        output_ = std::move(next);
        save_pcd_if_needed(output_);
        write_runtime_log(output_);
    }

    if (tf_buffer != nullptr) {
        tf_buffer->set_transform(ODOM_FRAME, BODY_FRAME, odom_T_body_for_tf, packet.timestamp_ns);
    }

    new_output_.store(true, std::memory_order_relaxed);
    return true;
}

void PointLio::configure_core() {
    p_pre.reset(new Preprocess());
    p_imu.reset(new ImuProcess());

    lidar_type = config_.lidar_type;
    imu_en = config_.imu_en;
    extrinsic_est_en = config_.extrinsic_est_en;
    use_imu_as_input = config_.use_imu_as_input;
    space_down_sample = config_.space_down_sample;
    publish_odometry_without_downsample = config_.publish_odometry_without_downsample;
    init_map_size = config_.init_map_size;
    match_s = config_.match_s;
    satu_acc = config_.satu_acc;
    satu_gyro = config_.satu_gyro;
    acc_norm = config_.acc_norm;
    plane_thr = static_cast<float>(config_.plane_thr);
    filter_size_surf_min = config_.filter_size_surf_min;
    filter_size_map_min = config_.filter_size_map_min;
    fov_deg = config_.fov_degree;
    DET_RANGE = static_cast<float>(config_.det_range);
    imu_time_inte = config_.imu_time_inte;
    lidar_time_inte = config_.lidar_time_inte;
    laser_point_cov = config_.lidar_meas_cov;
    acc_cov_input = config_.acc_cov_input;
    gyr_cov_input = config_.gyr_cov_input;
    vel_cov = 20.0;
    gyr_cov_output = config_.gyr_cov_output;
    acc_cov_output = config_.acc_cov_output;
    b_gyr_cov = config_.b_gyr_cov;
    b_acc_cov = config_.b_acc_cov;
    imu_meas_acc_cov = config_.imu_meas_acc_cov;
    imu_meas_omg_cov = config_.imu_meas_omg_cov;
    path_en = config_.path_en;
    scan_pub_en = config_.scan_publish_en;
    scan_body_pub_en = config_.scan_bodyframe_pub_en;
    pcd_save_en = config_.pcd_save_en;
    pcd_save_interval = config_.pcd_save_interval;
    runtime_pos_log = config_.runtime_pos_log;
    gravity = {config_.gravity.x(), config_.gravity.y(), config_.gravity.z()};
    gravity_init = {config_.gravity_init.x(), config_.gravity_init.y(), config_.gravity_init.z()};
    extrinT = {config_.extrinsic_t.x(), config_.extrinsic_t.y(), config_.extrinsic_t.z()};
    extrinR.resize(9);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            extrinR[static_cast<std::size_t>(r * 3 + c)] = config_.extrinsic_r(r, c);
        }
    }

    Lidar_T_wrt_IMU = config_.extrinsic_t;
    Lidar_R_wrt_IMU = config_.extrinsic_r;
    G_m_s2 = config_.gravity.norm();

    p_pre->lidar_type = config_.lidar_type;
    p_pre->N_SCANS = config_.scan_line;
    p_pre->blind = config_.blind;
    p_pre->det_range = config_.det_range;
    p_pre->point_filter_num = 1;
    p_imu->lidar_type = config_.lidar_type;
    p_imu->imu_en = config_.imu_en;
    p_imu->gravity_ = config_.gravity;

    ivox_options_.resolution_ = static_cast<float>(config_.ivox_grid_resolution);
    if (config_.ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (config_.ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (config_.ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    Eigen::Matrix<double, 24, 24> p_init;
    reset_cov(p_init);
    kf_input.change_P(p_init);
    Eigen::Matrix<double, 30, 30> p_init_output;
    reset_cov_output(p_init_output);
    kf_output.change_P(p_init_output);
    kf_input.x_.gravity = config_.gravity;
    kf_output.x_.gravity = config_.gravity;
    kf_input.x_.offset_R_L_I = config_.extrinsic_r;
    kf_input.x_.offset_T_L_I = config_.extrinsic_t;
    kf_output.x_.offset_R_L_I = config_.extrinsic_r;
    kf_output.x_.offset_T_L_I = config_.extrinsic_t;
}

void PointLio::save_pcd_if_needed(const PointLioOutput &output) {
    if (!config_.pcd_save_en || !output.has_laser_map || !output.laser_map || output.laser_map->empty()) {
        return;
    }

    std::filesystem::create_directories(PCD_DIR);
    *pcd_accumulated_ += *output.laser_map;
    ++pcd_frame_count_;

    if (config_.pcd_save_interval <= 0 || pcd_frame_count_ < config_.pcd_save_interval) {
        return;
    }

    const auto file = std::filesystem::path(PCD_DIR) / ("scans_" + std::to_string(pcd_save_index_++) + ".pcd");
    pcl::io::savePCDFileBinary(file.string(), *pcd_accumulated_);
    pcd_accumulated_->clear();
    pcd_frame_count_ = 0;
}

void PointLio::write_runtime_log(const PointLioOutput &output) {
    if (!config_.runtime_pos_log || !runtime_log_.is_open() || !output.has_odometry) {
        return;
    }

    const Eigen::Vector3d t = output.odometry.odom_T_body.translation();
    const Eigen::Quaterniond q(output.odometry.odom_T_body.linear());
    runtime_log_ << std::fixed << std::setprecision(9)
                 << static_cast<double>(output.timestamp_ns) * 1e-9 << ' '
                 << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
                 << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ' '
                 << output.odometry.velocity.x() << ' ' << output.odometry.velocity.y() << ' ' << output.odometry.velocity.z() << '\n';
}

} // namespace app::point_lio
