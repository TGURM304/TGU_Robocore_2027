#pragma once

#include <memory>

#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

namespace sensor_msgs {

struct Imu {
    using Ptr = std::shared_ptr<Imu>;
    using ConstPtr = std::shared_ptr<const Imu>;

    ros::Header header;
    geometry_msgs::Vector3 angular_velocity;
    geometry_msgs::Vector3 linear_acceleration;
};

} // namespace sensor_msgs
