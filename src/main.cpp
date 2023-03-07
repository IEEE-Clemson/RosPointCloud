#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/intersections.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
/*#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/organized_edge_detection.h>*/
// #include <pcl_ros/transforms.h>

enum class WallType
{
  NORTH,
  EAST,
  SOUTH,
  WEST,
};

using PointT = pcl::PointXYZRGB;
using namespace std::chrono_literals;
// using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    hasInitialized = false;
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mycloud", 10);
    tfBroad = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points",
                                                                             10, std::bind(&MinimalPublisher::pc_callback, this, std::placeholders::_1));
    tf_buffer =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

private:
  WallType classifyWallType(Eigen::Vector4f coeff)
  {
    Eigen::Vector3f norm = coeff.head<3>();
    float d = coeff[3];
    auto guess = tf_buffer->lookupTransform("camera_link", "map", tf2::TimePointZero);
    Eigen::Quaternionf rot_guess(guess.transform.rotation.w,
                                 guess.transform.rotation.x,
                                 guess.transform.rotation.y,
                                 guess.transform.rotation.z);
    Eigen::Vector3f trans_guess(guess.transform.translation.x,
                                guess.transform.translation.y,
                                guess.transform.translation.z);
    Eigen::Vector3f pos_guess = rot_guess * norm;
    Eigen::Vector3f localX = rot_guess * Eigen::Vector3f{1.0, 0.0, 0.0};
    Eigen::Vector3f localY = rot_guess * Eigen::Vector3f{0.0, 1.0, 0.0};
    if(fabs(pos_guess.y()) < fabs(pos_guess.x())) {
      // Either north or south wall
      if((pos_guess*d).dot(localX)  > 0) {
        return WallType::SOUTH;
      } else {
        return WallType::NORTH;
      }
    } else {
      // Either east or west
      if((pos_guess*d).dot(localY) > 0) {
        return WallType::WEST;
      } else {
        return WallType::EAST;
      }
    }
  }

  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!hasInitialized)
    {
      geometry_msgs::msg::TransformStamped tmsg;
      tmsg.header.stamp = this->get_clock()->now();
      tmsg.transform.translation.x = -0.5;
      tmsg.child_frame_id = "mytransform";
      tmsg.header.frame_id = "map";
      tfBroad->sendTransform(tmsg);

      tmsg.transform.translation.x = 0.0;
      tmsg.child_frame_id = "camera_link";
      tmsg.header.frame_id = "mytransform";
      tfBroad->sendTransform(tmsg);
      hasInitialized = true;
    }
    pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *temp_cloud);

    float minY = 0.03;
    float maxY = 0.06;

    // FILTER OUTSIDE OF RANGE
    pcl::PointCloud<PointT>::Ptr filter_cloud(new pcl::PointCloud<PointT>);
    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
    range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, minY)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, maxY)));

    pcl::ConditionalRemoval<PointT> range_filt;
    range_filt.setInputCloud(temp_cloud);
    range_filt.setCondition(range_cond);
    range_filt.filter(*filter_cloud);

    // PLANAR FITTING
    int nr_points = (int)filter_cloud->size();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(filter_cloud);

    std::vector<std::pair<size_t, pcl::ModelCoefficients::Ptr>> coefficients_list;
    pcl::PointCloud<PointT>::Ptr filter_cloud_temp(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane_cloud_temp(new pcl::PointCloud<PointT>);
    while (filter_cloud->size() > 0.1 * nr_points)
    {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      seg.setInputCloud(filter_cloud);
      seg.segment(*inliers, *coefficients);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filter_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      if (inliers->indices.size() > 100)
      {
        coefficients_list.push_back({inliers->indices.size(), coefficients});
        extract.filter(*plane_cloud_temp);
        *plane_cloud += *plane_cloud_temp;
      }

      // Remove Points that created plane
      extract.setNegative(true);
      extract.filter(*filter_cloud_temp);
      *filter_cloud = *filter_cloud_temp;
    }
    std::cout << "Found " << coefficients_list.size() << " valid planes" << std::endl;
    // FIND INTERSECTION OF PLANES
    // Find 2 biggest planes
    auto end = std::remove_if(coefficients_list.begin(), coefficients_list.end(), [](auto a)
                              { return fabs(a.second->values[1]) > 0.3; });
    if (end - coefficients_list.begin() >= 2)
    {
      std::partial_sort(coefficients_list.begin(),
                        coefficients_list.begin() + 2, end, [](auto a, auto b)
                        { return a.first > b.first; });
      auto coeffA = coefficients_list[0];
      auto coeffB = coefficients_list[1];
      Eigen::VectorXd line;
      Eigen::Vector4f a(coeffA.second->values.data());
      Eigen::Vector4f b(coeffB.second->values.data());
      // Verify the orthogonality of these planes
      bool orthogonal = fabs(a.head<3>().dot(b.head<3>())) < 0.2;
      bool couldFind = pcl::planeWithPlaneIntersection(a.cast<double>(), b.cast<double>(), line);
      if (couldFind && orthogonal)
      {
        auto pos = line.head<3>();
        auto dir = line.tail<3>();
        
        // Put point at y=0
        auto point = pos - (pos.y() / dir.y()) * dir;
        

        // Find closest corner based off of previous guess
        auto guess = tf_buffer->lookupTransform("camera_link", "map", tf2::TimePointZero);
        Eigen::Quaterniond rot_guess(guess.transform.rotation.w,
                                     guess.transform.rotation.x,
                                     guess.transform.rotation.y,
                                     guess.transform.rotation.z);
        Eigen::Vector3d trans_guess(guess.transform.translation.x,
                                    guess.transform.translation.y,
                                    guess.transform.translation.z);
        Eigen::Vector3d pos_guess = rot_guess * point + trans_guess;
        Eigen::Vector3d best_guess;
        double minDist = 999999;
        WallType wallA = classifyWallType(a);
        WallType wallB = classifyWallType(b);
        bool hasNorth = wallA == WallType::NORTH || wallB == WallType::NORTH;
        bool hasSouth = wallA == WallType::SOUTH || wallB == WallType::SOUTH;
        bool hasEast = wallA == WallType::EAST || wallB == WallType::EAST;
        bool hasWest = wallA == WallType::WEST || wallB == WallType::WEST;

        if(hasNorth && hasEast) {
          best_guess = {1.17 / 2, 0.0, -2.34 / 2};
        } else if(hasNorth && hasWest) {
          best_guess = {1.17 / 2, 0.0, 2.34 / 2};
        } else if(hasSouth && hasEast) {
          best_guess = {-1.17 / 2, 0.0, -2.34 / 2};
        } else if(hasSouth && hasWest) {
          best_guess = {-1.17 / 2, 0.0, 2.34 / 2};
        } else {
          return;
        }

        // Wall A is largest, compute orientation based off of it
        float initial_angle = 0;
        switch(wallA) {
          case WallType::NORTH:
            initial_angle = 0;
            break;
          case WallType::EAST:
            initial_angle = M_PI/2.0;
            break;
          case WallType::SOUTH:
            initial_angle = M_PI;
            break;
          case WallType::WEST:
            initial_angle = M_PI;
            break;
        }
        // Get angle from plane
        float theta;
        if(fabs(a[0]) > 0) {
          theta = atan2(a[0], a[2]);
        } else {
          theta = atan2(-a[0], -a[2]);
        }
        float new_angle =  theta+initial_angle - M_PI/2;
        std::cout << "Point: " << point << " " << "Theta: " << theta << "Wall A: " << (int) wallA << "Wall B: " <<(int)wallB <<  std::endl;
        Eigen::Quaterniond new_rot (Eigen::AngleAxisd(new_angle, Eigen::Vector3d{0, 0, 1}));
        auto rot_point = new_rot*Eigen::Vector3d{point.x(), point.z(), 0.0};
        geometry_msgs::msg::TransformStamped tmsg;
        tmsg.header.stamp = this->get_clock()->now();
        tmsg.header.frame_id = "/mytransform";
        tmsg.child_frame_id = "/camera_link";
        tmsg.transform.translation.x = -rot_point.y();
        tmsg.transform.translation.y = rot_point.x();
        tmsg.transform.translation.z = 0.0;
        tmsg.transform.rotation.w = new_rot.w();
        tmsg.transform.rotation.x = new_rot.x();
        tmsg.transform.rotation.y = new_rot.y();
        tmsg.transform.rotation.z = new_rot.z();
        tfBroad->sendTransform(tmsg);

        geometry_msgs::msg::TransformStamped tmsg2;
        // tmsg.header.stamp = this->get_clock()->now();
        tmsg.header.frame_id = "/map";
        tmsg.child_frame_id = "/mytransform";
        tmsg.transform.translation.x = best_guess.x();
        tmsg.transform.translation.y = best_guess.z();
        tmsg.transform.translation.z = 0.0;
        tmsg.transform.rotation.w = 1.0;
        tmsg.transform.rotation.x = 0.0;
        tmsg.transform.rotation.y = 0.0;
        tmsg.transform.rotation.z = 0.0;
        tfBroad->sendTransform(tmsg);
      }
    }
    // PUBLISHING
    sensor_msgs::msg::PointCloud2 pubMsg;
    pcl::toROSMsg(*plane_cloud, pubMsg);
    pubMsg.header.frame_id = msg->header.frame_id;
    pubMsg.header.stamp = msg->header.stamp;
    publisher_->publish(pubMsg);
  }

  // Commented out to improve compile times
  /*void pc_callback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    float th_dd = 0.01;
    int max_search = 10;
    pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *temp_cloud);

    float minY = -0.03;
    float maxY = 0.03;

    // FILTER OUTSIDE OF RANGE
    pcl::PointCloud<PointT>::Ptr filter_cloud(new pcl::PointCloud<PointT>);
    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
    range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, minY)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, maxY)));

    pcl::ConditionalRemoval<PointT> range_filt;
    range_filt.setInputCloud(temp_cloud);
    range_filt.setCondition(range_cond);
    range_filt.setKeepOrganized(true);
    range_filt.filter(*filter_cloud);

    // NORMAL DETECTION
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setNormalSmoothingSize(10.0f);
    ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
    ne.setInputCloud(filter_cloud);
    ne.compute(*normal);
    // EDGE DETECTION
    pcl::OrganizedEdgeFromNormals<PointT, pcl::Normal, pcl::Label> oed;
    // OrganizedEdgeFromRGBNormals<PointXYZ, Normal, Label> oed;
    oed.setInputNormals(normal);
    oed.setInputCloud(filter_cloud);
    oed.setDepthDisconThreshold(th_dd);
    //oed.setMaxSearchNeighbors (max_search);
    oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED);
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> label_indices;
    oed.compute(labels, label_indices);
    pcl::PointCloud<PointT>::Ptr occluding_edges(new pcl::PointCloud<PointT>),
        occluded_edges(new pcl::PointCloud<PointT>),
        nan_boundary_edges(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*filter_cloud, label_indices[0].indices, *nan_boundary_edges);
    pcl::copyPointCloud(*filter_cloud, label_indices[1].indices, *occluding_edges);
    pcl::copyPointCloud(*filter_cloud, label_indices[2].indices, *occluded_edges);
    // PUBLISHING
    sensor_msgs::msg::PointCloud2 pubMsg;
    pcl::toROSMsg(*nan_boundary_edges, pubMsg);
    pubMsg.header.frame_id = msg->header.frame_id;
    pubMsg.header.stamp = msg->header.stamp;
    publisher_->publish(pubMsg);
  }*/

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroad;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  size_t count_;
  bool hasInitialized;
  std::vector<Eigen::Vector3d> corners = {
      {1.17 / 2, 0.0, 2.34 / 2},
      {
          -1.17 / 2,
          0.0,
          2.34 / 2,
      },
      {-1.17 / 2, 0.0, 2.34 / 2},
      {1.17 / 2, 0.0, -2.34 / 2},
  };
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
