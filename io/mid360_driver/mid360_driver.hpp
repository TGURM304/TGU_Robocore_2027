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
#include <vector>

#include "tools/BS_thread_pool.hpp"

namespace io {

    struct Point {
        double timestamp;
        double offset_time;
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
    public:
        using PointCloudCallback = std::function<void(const asio::ip::address &lidar_ip,
                                                       const std::vector<Point> &points,
                                                       uint64_t timestamp_ns)>;
        using ImuCallback = std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)>;

    private:
        std::atomic<bool> is_running = true;
        std::string_view cfg_file_path_;
        asio::ip::address host_ip;
        asio::ip::udp::socket receive_pointcloud_socket;
        asio::ip::udp::socket receive_imu_socket;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> delta_time_map;
        PointCloudCallback on_receive_pointcloud;
        ImuCallback on_receive_imu;
        BS::thread_pool<> pointcloud_callback_pool;
        BS::thread_pool<> imu_callback_pool;

    public:
        Mid360Driver(asio::io_context &io_context,
                     std::string_view cfg_file_path,
                     PointCloudCallback on_receive_pointcloud,
                     ImuCallback on_receive_imu,
                     std::size_t pointcloud_callback_thread_count = 1,
                     std::size_t imu_callback_thread_count = 1);

        ~Mid360Driver();

        void stop();

        asio::awaitable<void> receive_pointcloud();

        asio::awaitable<void> receive_imu();
    };

}// namespace io
