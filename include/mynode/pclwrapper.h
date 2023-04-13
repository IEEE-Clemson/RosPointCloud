#pragma once

#ifndef DO_NOT_INCLUDE_STRUCT
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
using PointT = pcl::PointXYZHSV;
struct PCLWrapper {
    pcl::PointCloud<PointT>::Ptr transformed_cloud;
    rclcpp::Time time;
};
#else
struct PCLWrapper;
#endif

extern void createPCLWrapper(PCLWrapper** wrapper);

struct MinimalPublisher;
extern void transformCloud(MinimalPublisher& ctx, PCLWrapper* wrapper, const sensor_msgs::msg::PointCloud2::SharedPtr msg);