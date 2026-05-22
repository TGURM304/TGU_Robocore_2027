#pragma once

#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

namespace nav_msgs {

struct Odometry {
    ros::Header header;
    std::string child_frame_id;
    struct PoseWithCovariance {
        geometry_msgs::Pose pose;
    } pose;
};

} // namespace nav_msgs
