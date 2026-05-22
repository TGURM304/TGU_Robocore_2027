/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "mid360_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

#include "tools/tomlpp.hpp"

namespace io {

    namespace {

#ifdef MID360_DRIVER_USE_BOOST_ASIO
        using ErrorCode = boost::system::error_code;
#else
        using ErrorCode = asio::error_code;
#endif

        constexpr double kPi = 3.141592653589793238462643383279502884;
        constexpr std::size_t kBufferSize = 1400;
        constexpr std::uint16_t kLidarPointcloudSourcePort = 56300;
        constexpr std::uint16_t kHostPointcloudReceivePort = 56301;
        constexpr std::uint16_t kLidarImuSourcePort = 56400;
        constexpr std::uint16_t kHostImuReceivePort = 56401;

        enum DataType : std::uint8_t {
            kLivoxLidarImuData = 0,
            kLivoxLidarCartesianCoordinateHighData = 0x01,
            kLivoxLidarCartesianCoordinateLowData = 0x02,
            kLivoxLidarSphericalCoordinateData = 0x03
        };

        enum TimestampType : std::uint8_t {
            kTimestampTypeNoSync = 0,
            kTimestampTypeGptpOrPtp = 1,
            kTimestampTypeGps = 2
        };

#pragma pack(1)

        struct DataHeader {
            std::uint8_t version;
            std::uint16_t length;
            std::uint16_t time_interval;
            std::uint16_t dot_num;
            std::uint16_t udp_cnt;
            std::uint8_t frame_cnt;
            DataType data_type;
            TimestampType time_type;
            std::uint8_t reserved[12];
            std::uint32_t crc32;
            std::uint64_t timestamp;
        };

        struct Imu {
            float angular_velocity_x;
            float angular_velocity_y;
            float angular_velocity_z;
            float linear_acceleration_x;
            float linear_acceleration_y;
            float linear_acceleration_z;
        };

        struct CartesianHighPoint {
            std::int32_t x;
            std::int32_t y;
            std::int32_t z;
            std::uint8_t reflectivity;
            std::uint8_t tag;
        };

        struct CartesianLowPoint {
            std::int16_t x;
            std::int16_t y;
            std::int16_t z;
            std::uint8_t reflectivity;
            std::uint8_t tag;
        };

        struct SphericalPoint {
            std::uint32_t depth;
            std::uint16_t theta;
            std::uint16_t phi;
            std::uint8_t reflectivity;
            std::uint8_t tag;
        };

#pragma pack()

        void combine_4_bytes(std::size_t &seed, const unsigned char *bytes) {
            const std::size_t bytes_hash =
                    (static_cast<std::size_t>(bytes[0]) << 24) |
                    (static_cast<std::size_t>(bytes[1]) << 16) |
                    (static_cast<std::size_t>(bytes[2]) << 8) |
                    (static_cast<std::size_t>(bytes[3]));
            seed ^= bytes_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        bool is_valid_point_tag(std::uint8_t tag) {
            return (tag & 0b00110000) == 0b00000000 &&
                   (tag & 0b00001100) == 0b00000000 &&
                   (tag & 0b00000011) == 0b00000000;
        }

        double current_time_seconds() {
            return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        }

        uint64_t to_timestamp_ns(double timestamp_s) {
            return static_cast<uint64_t>(timestamp_s * 1e9);
        }

        bool has_payload_size(std::size_t received_size, std::size_t expected_payload_size) {
            return received_size >= sizeof(DataHeader) && received_size - sizeof(DataHeader) >= expected_payload_size;
        }

        double point_offset_seconds(const DataHeader &header, std::size_t point_index) {
            return static_cast<double>(header.time_interval) * static_cast<double>(point_index) * 1e-9;
        }

    }// namespace

    std::size_t IpAddressHasher::operator()(const asio::ip::address &addr) const noexcept {
        if (addr.is_v4()) {
            return std::hash<unsigned int>()(addr.to_v4().to_uint());
        }

        const asio::ip::address_v6::bytes_type bytes = addr.to_v6().to_bytes();
        std::size_t result = static_cast<std::size_t>(addr.to_v6().scope_id());
        combine_4_bytes(result, &bytes[0]);
        combine_4_bytes(result, &bytes[4]);
        combine_4_bytes(result, &bytes[8]);
        combine_4_bytes(result, &bytes[12]);
        return result;
    }

    Mid360Driver::Mid360Driver(asio::io_context &io_context,
                               std::string_view cfg_file_path,
                               PointCloudCallback on_receive_pointcloud,
                               ImuCallback on_receive_imu,
                               std::size_t pointcloud_callback_thread_count,
                               std::size_t imu_callback_thread_count)
        : cfg_file_path_(cfg_file_path),
          receive_pointcloud_socket(io_context),
          receive_imu_socket(io_context),
          on_receive_pointcloud(std::move(on_receive_pointcloud)),
          on_receive_imu(std::move(on_receive_imu)),
          pointcloud_callback_pool(pointcloud_callback_thread_count),
          imu_callback_pool(imu_callback_thread_count) {
        if (!cfg_file_path_.empty()) {
            auto config = toml::parse_file(std::string(cfg_file_path_));
            host_ip = asio::ip::make_address(config["mid360_driver"]["host_ip"].value_or("192.168.1.50"));
        } else {
            host_ip = asio::ip::make_address("192.168.1.50");
        }
        receive_pointcloud_socket.open(asio::ip::udp::v4());
        receive_pointcloud_socket.bind(asio::ip::udp::endpoint(host_ip, kHostPointcloudReceivePort));
        receive_imu_socket.open(asio::ip::udp::v4());
        receive_imu_socket.bind(asio::ip::udp::endpoint(host_ip, kHostImuReceivePort));
        co_spawn(io_context, receive_pointcloud(), asio::detached);
        co_spawn(io_context, receive_imu(), asio::detached);
    }

    Mid360Driver::~Mid360Driver() {
        stop();
    }

    void Mid360Driver::stop() {
        is_running.store(false, std::memory_order_relaxed);
        ErrorCode error_code;
        receive_pointcloud_socket.close(error_code);
        receive_imu_socket.close(error_code);
        pointcloud_callback_pool.wait();
        imu_callback_pool.wait();
    }

    asio::awaitable<void> Mid360Driver::receive_pointcloud() {
        std::uint8_t buffer[kBufferSize];
        asio::ip::udp::endpoint sender_endpoint;
        std::vector<Point> points;
        while (is_running.load(std::memory_order_relaxed)) {
            ErrorCode error_code;
            const std::size_t received_size = co_await receive_pointcloud_socket.async_receive_from(
                    asio::buffer(buffer, kBufferSize),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, error_code));
            if (error_code || sender_endpoint.port() != kLidarPointcloudSourcePort || received_size < sizeof(DataHeader)) [[unlikely]] {
                continue;
            }

            const auto &header = *reinterpret_cast<const DataHeader *>(buffer);
            double header_timestamp = static_cast<double>(header.timestamp) * 1e-9;
            if (header.time_type == TimestampType::kTimestampTypeNoSync) {
                auto [iter, inserted] = delta_time_map.try_emplace(sender_endpoint.address());
                if (inserted) {
                    const double now = current_time_seconds();
                    iter->second = now - header_timestamp;
                    header_timestamp = now;
                } else {
                    header_timestamp += iter->second;
                }
            }

            const uint64_t timestamp_ns = to_timestamp_ns(header_timestamp);
            points.clear();
            points.reserve(header.dot_num);
            if (header.data_type == DataType::kLivoxLidarCartesianCoordinateHighData) {
                if (!has_payload_size(received_size, static_cast<std::size_t>(header.dot_num) * sizeof(CartesianHighPoint))) {
                    continue;
                }
                const auto *raw_points = reinterpret_cast<const CartesianHighPoint *>(buffer + sizeof(DataHeader));
                for (std::size_t i = 0; i < header.dot_num; ++i) {
                    const auto &raw_point = raw_points[i];
                    if (!is_valid_point_tag(raw_point.tag)) {
                        continue;
                    }
                    const double offset_time = point_offset_seconds(header, i);
                    Point point{};
                    point.timestamp = header_timestamp + offset_time;
                    point.offset_time = offset_time;
                    point.x = static_cast<float>(raw_point.x * 0.001);
                    point.y = static_cast<float>(raw_point.y * 0.001);
                    point.z = static_cast<float>(raw_point.z * 0.001);
                    point.intensity = static_cast<float>(raw_point.reflectivity);
                    points.push_back(point);
                }
            } else if (header.data_type == DataType::kLivoxLidarCartesianCoordinateLowData) {
                if (!has_payload_size(received_size, static_cast<std::size_t>(header.dot_num) * sizeof(CartesianLowPoint))) {
                    continue;
                }
                const auto *raw_points = reinterpret_cast<const CartesianLowPoint *>(buffer + sizeof(DataHeader));
                for (std::size_t i = 0; i < header.dot_num; ++i) {
                    const auto &raw_point = raw_points[i];
                    if (!is_valid_point_tag(raw_point.tag)) {
                        continue;
                    }
                    const double offset_time = point_offset_seconds(header, i);
                    Point point{};
                    point.timestamp = header_timestamp + offset_time;
                    point.offset_time = offset_time;
                    point.x = static_cast<float>(raw_point.x * 0.001);
                    point.y = static_cast<float>(raw_point.y * 0.001);
                    point.z = static_cast<float>(raw_point.z * 0.001);
                    point.intensity = static_cast<float>(raw_point.reflectivity);
                    points.push_back(point);
                }
            } else if (header.data_type == DataType::kLivoxLidarSphericalCoordinateData) {
                if (!has_payload_size(received_size, static_cast<std::size_t>(header.dot_num) * sizeof(SphericalPoint))) {
                    continue;
                }
                const auto *raw_points = reinterpret_cast<const SphericalPoint *>(buffer + sizeof(DataHeader));
                for (std::size_t i = 0; i < header.dot_num; ++i) {
                    const auto &raw_point = raw_points[i];
                    if (!is_valid_point_tag(raw_point.tag)) {
                        continue;
                    }
                    const double offset_time = point_offset_seconds(header, i);
                    Point point{};
                    point.timestamp = header_timestamp + offset_time;
                    point.offset_time = offset_time;
                    const double radius = raw_point.depth / 1000.0;
                    const double theta = raw_point.theta / 100.0 / 180.0 * kPi;
                    const double phi = raw_point.phi / 100.0 / 180.0 * kPi;
                    point.x = static_cast<float>(radius * std::sin(theta) * std::cos(phi));
                    point.y = static_cast<float>(radius * std::sin(theta) * std::sin(phi));
                    point.z = static_cast<float>(radius * std::cos(theta));
                    point.intensity = static_cast<float>(raw_point.reflectivity);
                    points.push_back(point);
                }
            }
            const asio::ip::address lidar_ip = sender_endpoint.address();
            pointcloud_callback_pool.detach_task([this, lidar_ip, points = std::move(points), timestamp_ns] {
                on_receive_pointcloud(lidar_ip, points, timestamp_ns);
            });
        }
        co_return;
    }

    asio::awaitable<void> Mid360Driver::receive_imu() {
        std::uint8_t buffer[kBufferSize];
        asio::ip::udp::endpoint sender_endpoint;
        while (is_running.load(std::memory_order_relaxed)) {
            ErrorCode error_code;
            const std::size_t received_size = co_await receive_imu_socket.async_receive_from(
                    asio::buffer(buffer, kBufferSize),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, error_code));
            if (error_code || sender_endpoint.port() != kLidarImuSourcePort || !has_payload_size(received_size, sizeof(Imu))) [[unlikely]] {
                continue;
            }

            const auto &header = *reinterpret_cast<const DataHeader *>(buffer);
            double header_timestamp = static_cast<double>(header.timestamp) * 1e-9;
            if (header.time_type == TimestampType::kTimestampTypeNoSync) {
                auto [iter, inserted] = delta_time_map.try_emplace(sender_endpoint.address());
                if (inserted) {
                    const double now = current_time_seconds();
                    iter->second = now - header_timestamp;
                    header_timestamp = now;
                } else {
                    header_timestamp += iter->second;
                }
            }

            const auto &raw_imu = *reinterpret_cast<const Imu *>(buffer + sizeof(DataHeader));
            ImuMsg imu_msg{};
            imu_msg.timestamp = header_timestamp;
            imu_msg.angular_velocity_x = raw_imu.angular_velocity_x;
            imu_msg.angular_velocity_y = raw_imu.angular_velocity_y;
            imu_msg.angular_velocity_z = raw_imu.angular_velocity_z;
            imu_msg.linear_acceleration_x = raw_imu.linear_acceleration_x;
            imu_msg.linear_acceleration_y = raw_imu.linear_acceleration_y;
            imu_msg.linear_acceleration_z = raw_imu.linear_acceleration_z;
            const asio::ip::address lidar_ip = sender_endpoint.address();
            imu_callback_pool.detach_task([this, lidar_ip, imu_msg] {
                on_receive_imu(lidar_ip, imu_msg);
            });
        }
        co_return;
    }

}// namespace io
