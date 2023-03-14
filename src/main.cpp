#include "mynode/main.h"
#include "mynode/pointcloudodom.h"

void MinimalPublisher::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  handleOdom(*this, msg); 
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
