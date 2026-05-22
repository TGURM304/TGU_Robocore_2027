#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include "sensor_msgs/PointCloud2.h"

namespace pcl {

template <typename PointT>
void fromROSMsg(const sensor_msgs::PointCloud2&, pcl::PointCloud<PointT>& cloud) {
    cloud.clear();
}

template <typename PointT>
void toROSMsg(const pcl::PointCloud<PointT>&, sensor_msgs::PointCloud2&) {}

} // namespace pcl
