#pragma once

#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

namespace nav_msgs {

struct Path {
    ros::Header header;
    std::vector<geometry_msgs::PoseStamped> poses;
};

} // namespace nav_msgs
