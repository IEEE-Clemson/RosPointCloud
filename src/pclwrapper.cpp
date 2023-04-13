#include "mynode/pclwrapper.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "mynode/main.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>

void createPCLWrapper(PCLWrapper** wrapper) {
    *wrapper = new PCLWrapper;
    (*wrapper)->transformed_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

void transformCloud(MinimalPublisher& ctx, PCLWrapper* wrapper, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    wrapper->time = msg->header.stamp;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    
    pcl::PointCloud<PointT>::Ptr hsv_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloudXYZRGBtoXYZHSV(*temp_cloud, *hsv_cloud);
    pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);

    geometry_msgs::msg::TransformStamped camera_rot = ctx.tf_buffer->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    Eigen::Affine3f cam_trans = Eigen::Affine3f::Identity();
    cam_trans.rotate(Eigen::Quaternionf(camera_rot.transform.rotation.w,
                       camera_rot.transform.rotation.x,
                        camera_rot.transform.rotation.y,
                         camera_rot.transform.rotation.z));
    pcl::transformPointCloud(*hsv_cloud, *transformed_cloud, cam_trans);
    float minY = -0.18;
    float maxY = -0.13;

    // FILTER OUTSIDE OF RANGE
    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
    range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, minY)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, maxY)));

    pcl::ConditionalRemoval<PointT> range_filt;
    range_filt.setInputCloud(transformed_cloud);
    range_filt.setCondition(range_cond);
    range_filt.filter(*wrapper->transformed_cloud);
}