#include "mynode/main.h"
#include "mynode/pclwrapper.h"

#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
void handleObjects(MinimalPublisher &ctx, PCLWrapper *wrapper)
{
  using HSVPoint = pcl::PointXYZHSV;
  // Convert to hsv
  pcl::PointCloud<HSVPoint>::Ptr hsv_cloud(new pcl::PointCloud<HSVPoint>);
  pcl::PointCloudXYZRGBtoXYZHSV(*wrapper->transformed_cloud, *hsv_cloud);

  // Filter based off color
  // FILTER OUTSIDE OF RANGE
  pcl::ConditionAnd<HSVPoint>::Ptr range_cond(new pcl::ConditionAnd<HSVPoint>());
  //range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("s", pcl::ComparisonOps::GT, 0)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("s", pcl::ComparisonOps::LT, 0.2)));
  range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("v", pcl::ComparisonOps::GT, 0.5)));
  //range_cond->addComparison(pcl::FieldComparison<HSVPoint>::Ptr(new pcl::FieldComparison<HSVPoint>("v", pcl::ComparisonOps::LT, maxY)));

  pcl::ConditionalRemoval<HSVPoint> range_filt;
  range_filt.setInputCloud(hsv_cloud);
  range_filt.setCondition(range_cond);
  pcl::PointCloud<HSVPoint>::Ptr filtered_hsv(new pcl::PointCloud<HSVPoint>);
  range_filt.filter(*filtered_hsv);

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
  for(const auto& cluster : cluster_indices) {
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*filtered_hsv, cluster, centroid);
    //std::cout << "centroid: " << centroid << std::endl;
  }
  //std::cout << std::endl << std::endl;
}