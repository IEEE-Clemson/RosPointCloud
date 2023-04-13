#include "mynode/main.h"
#include "mynode/pclwrapper.h"

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

void findDuckHeads(MinimalPublisher &ctx, PCLWrapper *wrapper);
void findPedestal(MinimalPublisher &ctx, PCLWrapper *wrapper, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub, float hl, float sl, float vl, float hh, float sh, float vh);

void handleObjects(MinimalPublisher &ctx, PCLWrapper *wrapper)
{
  findDuckHeads(ctx, wrapper);
  // White Pedestal
  findPedestal(ctx, wrapper, ctx.whiteCylinderPose, 0.0, 0.0, 0.4, 360.0, 0.2, 1.0);
  // Green Pedestal
  findPedestal(ctx, wrapper, ctx.greenCylinderPose, 85.0, 0.7, 0.3, 120.0, 1.0, 1.0);
  // Red pedestal
  findPedestal(ctx, wrapper, ctx.redCylinderPose, 330.0, 0.6, 0.1, 360.0, 1.0, 1.0);
  //std::cout << std::endl << std::endl;
}

void findPedestal(MinimalPublisher &ctx, PCLWrapper *wrapper, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub, float hl, float sl, float vl, float hh, float sh, float vh) {
  using HSVPoint = pcl::PointXYZHSV;
  // Convert to hsv
  pcl::PointCloud<HSVPoint>::Ptr hsv_cloud = wrapper->transformed_cloud;//(new pcl::PointCloud<HSVPoint>);
  //pcl::PointCloudXYZRGBtoXYZHSV(*wrapper->transformed_cloud, *hsv_cloud);

  // Filter based off color
  // FILTER OUTSIDE OF RANGE
  pcl::ConditionAnd<HSVPoint>::Ptr range_cond(new pcl::ConditionAnd<HSVPoint>());
  //range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("s", pcl::ComparisonOps::GT, 0)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("h", pcl::ComparisonOps::LT, hh)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("h", pcl::ComparisonOps::GT, hl)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("s", pcl::ComparisonOps::LT, sh)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("s", pcl::ComparisonOps::GT, sl)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("v", pcl::ComparisonOps::LT, vh)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("v", pcl::ComparisonOps::GT, vl)));
  //range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("v", pcl::ComparisonOps::LT, maxY)));

  pcl::ConditionalRemoval<HSVPoint> range_filt;
  range_filt.setInputCloud(hsv_cloud);
  range_filt.setCondition(range_cond);
  pcl::PointCloud<HSVPoint>::Ptr filtered_hsv(new pcl::PointCloud<HSVPoint>);
  range_filt.filter(*filtered_hsv);
  if(filtered_hsv->size() == 0) return;
  // Euclidean clustering
  pcl::search::KdTree<HSVPoint>::Ptr tree(new pcl::search::KdTree<HSVPoint>);
  tree->setInputCloud(filtered_hsv);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<HSVPoint> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered_hsv);
  ec.extract(cluster_indices);
  //std::cout << "here" << std::endl;
  std::vector<geometry_msgs::msg::Pose> poses;
  for(const auto& cluster : cluster_indices) {
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*filtered_hsv, cluster, centroid);
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    pose.position.x = centroid.x();
    pose.position.y = centroid.y();
    pose.position.z = 0;

    poses.push_back(pose);
  }
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = wrapper->time;
  pose_array.header.frame_id = target_frame;
  pose_array.poses = poses;

  pub->publish(pose_array);
}

void findDuckHeads(MinimalPublisher &ctx, PCLWrapper *wrapper) {
  using HSVPoint = pcl::PointXYZHSV;

  pcl::PointCloud<HSVPoint>::Ptr hsv_cloud = wrapper->transformed_cloud;
  pcl::ConditionAnd<HSVPoint>::Ptr range_cond(new pcl::ConditionAnd<HSVPoint>());
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("z", pcl::ComparisonOps::GT, -0.16)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("h", pcl::ComparisonOps::GT, 40)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("h", pcl::ComparisonOps::LT, 60)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("s", pcl::ComparisonOps::GT, 0.7)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("v", pcl::ComparisonOps::GT, 0.5)));
  //range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("v", pcl::ComparisonOps::LT, maxY)));

  pcl::ConditionalRemoval<HSVPoint> range_filt;
  range_filt.setInputCloud(hsv_cloud);
  range_filt.setCondition(range_cond);
  pcl::PointCloud<HSVPoint>::Ptr filtered_hsv(new pcl::PointCloud<HSVPoint>);
  range_filt.filter(*filtered_hsv);
  
  if(filtered_hsv->size() == 0) return;

  pcl::search::KdTree<HSVPoint>::Ptr tree(new pcl::search::KdTree<HSVPoint>);
  tree->setInputCloud(filtered_hsv);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<HSVPoint> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered_hsv);
  ec.extract(cluster_indices);

  std::vector<geometry_msgs::msg::Pose> head_poses;
  std::vector<geometry_msgs::msg::Pose> upside_down_ducks;
  pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
  for(const auto& cluster : cluster_indices) {
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*filtered_hsv, cluster, centroid);
    geometry_msgs::msg::Pose pose;

    // Compute eccentricity 
    feature_extractor.setInputCloud(filtered_hsv);
    pcl::PointIndicesConstPtr ptr(new pcl::PointIndices(cluster));
    feature_extractor.setIndices(ptr);
    float major, middle, minor;
    std::cout << major << " " << middle << " " << minor << std::endl;
    feature_extractor.compute();
    feature_extractor.getEigenValues(major, middle, minor);
    // Identity quaternion orientation
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    pose.position.x = centroid.x();
    pose.position.y = centroid.y();
    pose.position.z = 0;

    // Major eigen value is very low value for upright ducks
    if(major < 0.00015) {
      head_poses.push_back(pose);
    } else {
      upside_down_ducks.push_back(pose);
    }
  }
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = wrapper->time;
  pose_array.header.frame_id = target_frame;
  pose_array.poses = head_poses;
  ctx.duckHeadPose->publish(pose_array);
  pose_array.poses = upside_down_ducks;
  ctx.flippedDuckPose->publish(pose_array);
}