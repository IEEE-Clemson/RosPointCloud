#include "mynode/main.h"
#include "mynode/pointcloudodom.h"
#include "mynode/duckfinder.h"

#define DO_NOT_INCLUDE_STRUCT
#include "mynode/pclwrapper.h"

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0)
  {
    hasInitialized = false;
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_cloud_name, 10);
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_name, 10);
    tfBroad = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(in_cloud_name,
                                                                             1, std::bind(&MinimalPublisher::pc_callback, this, std::placeholders::_1));
    tf_buffer =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    duckHeadPose = this->create_publisher<geometry_msgs::msg::PoseArray>("/pcl/duckheads", 10);
    flippedDuckPose = this->create_publisher<geometry_msgs::msg::PoseArray>("/pcl/flippedducks", 10);
    redCylinderPose = this->create_publisher<geometry_msgs::msg::PoseArray>("/pcl/redCylinderPose", 10);
    greenCylinderPose = this->create_publisher<geometry_msgs::msg::PoseArray>("/pcl/greenCylinderPose", 10);
    whiteCylinderPose = this->create_publisher<geometry_msgs::msg::PoseArray>("/pcl/whiteCylinderPose", 10);
    createPCLWrapper(&wrapper);
  }

void MinimalPublisher::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  transformCloud(*this, wrapper, msg);
  handleOdom(*this, wrapper);
  handleObjects(*this, wrapper);
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
