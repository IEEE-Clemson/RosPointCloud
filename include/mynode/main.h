#pragma once
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
//#include <pcl/common/common.h>

enum class WallType
{
  NORTH,
  EAST,
  SOUTH,
  WEST,
  NONE,
};


using namespace std::chrono_literals;

constexpr bool use_seperate_transform = false;
constexpr bool publish_transform = false;
inline std::string out_cloud_name = "/mycloud";
inline std::string in_cloud_name = "/camera/depth/color/points";
inline std::string target_frame = "camorg";
inline std::string map_frame = "map";
inline std::string odom_name = "/pcl/odom";

struct PCLWrapper;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher();
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  PCLWrapper *wrapper;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroad;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr duckHeadPose;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr flippedDuckPose;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr greenCylinderPose;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr redCylinderPose;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr whiteCylinderPose;
  size_t count_;
  bool hasInitialized;
};