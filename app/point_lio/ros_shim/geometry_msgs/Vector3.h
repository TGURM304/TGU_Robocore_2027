#pragma once

#include "ros/ros.h"

namespace geometry_msgs {

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct Point {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Pose {
    Point position;
    Quaternion orientation;
};

struct PoseStamped {
    ros::Header header;
    Pose pose;
};

} // namespace geometry_msgs
