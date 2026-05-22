/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#ifdef MID360_DRIVER_USE_BOOST_ASIO
#include <utility>

#include <boost/asio.hpp>
namespace asio = boost::asio;
#else
#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif

#ifndef ASIO_NO_DEPRECATED
#define ASIO_NO_DEPRECATED
#endif

#include <utility>

#include <asio.hpp>
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>
#include <unordered_map>

namespace io {

    struct Point {
        double timestamp;
        float x, y, z;
        float intensity;
    };

    struct ImuMsg {
        double timestamp;
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct IpAddressHasher {
        std::size_t operator()(const asio::ip::address &addr) const noexcept;
    };

    class Mid360Driver {
    private:
        std::atomic<bool> is_running = true;
        std::string_view cfg_file_path_;
        asio::ip::address host_ip;
        asio::ip::udp::socket receive_pointcloud_socket;
        asio::ip::udp::socket receive_imu_socket;
        std::vector<Point> points;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> delta_time_map;
        std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points, uint64_t timestamp_ns)> on_receive_pointcloud;
        std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu;

    public:
        Mid360Driver(asio::io_context &io_context,
                     std::string_view cfg_file_path,
                     const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points, uint64_t timestamp_ns)> &on_receive_pointcloud,
                     const std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> &on_receive_imu);

        ~Mid360Driver();

        void stop();

        asio::awaitable<void> receive_pointcloud();

        asio::awaitable<void> receive_imu();
    };

}// namespace io
