#pragma once
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
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


class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    hasInitialized = false;
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_cloud_name, 10);
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_name, 10);
    tfBroad = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(in_cloud_name,
                                                                             10, std::bind(&MinimalPublisher::pc_callback, this, std::placeholders::_1));
    tf_buffer =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroad;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
  size_t count_;
  bool hasInitialized;
};