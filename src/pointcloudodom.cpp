#include "mynode/pointcloudodom.h"

#include "mynode/pclwrapper.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>


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
#include <pcl/common/transforms.h>



  WallType classifyWallType(MinimalPublisher& ctx, Eigen::Vector4f coeff)
  {
    Eigen::Vector3f norm = coeff.head<3>();
    float d = coeff[3];
    if(!ctx.tf_buffer->canTransform(target_frame, map_frame, tf2::TimePointZero)) return WallType::NONE;
    auto guess = ctx.tf_buffer->lookupTransform(target_frame, map_frame, tf2::TimePointZero);
    Eigen::Quaternionf rot_guess(guess.transform.rotation.w,
                                 guess.transform.rotation.x,
                                 guess.transform.rotation.y,
                                 guess.transform.rotation.z);
    Eigen::Vector3f trans_guess(guess.transform.translation.x,
                                guess.transform.translation.y,
                                guess.transform.translation.z);
    Eigen::Vector3f norm_transform = {norm.x(), norm.y(), 0.0};
    Eigen::Vector3f pos_guess = rot_guess.conjugate() * norm_transform;
    if(fabs(pos_guess.y()) < fabs(pos_guess.x())) {
      // Either north or south wall
      if((pos_guess*d).x() < 0) {
        return WallType::NORTH;
      } else {
        return WallType::SOUTH;
      }
    } else {
      // Either east or west
      if((pos_guess*d).y() < 0) {
        return WallType::WEST;
      } else {
        return WallType::EAST;
      }
    }
  }

void handleOdom(MinimalPublisher& ctx, PCLWrapper* wrapper)  {
    if (!ctx.hasInitialized && publish_transform)
    {
      if(use_seperate_transform) {
      geometry_msgs::msg::TransformStamped tmsg;
        tmsg.header.stamp = ctx.get_clock()->now();
        tmsg.transform.translation.x = -0.5;
        tmsg.child_frame_id = "mytransform";
        tmsg.header.frame_id = map_frame;
        ctx.tfBroad->sendTransform(tmsg);

        tmsg.transform.translation.x = 0.0;
        tmsg.child_frame_id = target_frame;
        tmsg.header.frame_id = "mytransform";
        ctx.tfBroad->sendTransform(tmsg);
      } else {
        geometry_msgs::msg::TransformStamped tmsg;
        tmsg.header.stamp = ctx.get_clock()->now();
        tmsg.transform.translation.x = -0.5;
        tmsg.child_frame_id = target_frame;
        tmsg.header.frame_id = map_frame;
        ctx.tfBroad->sendTransform(tmsg);
        
      }
      ctx.hasInitialized = true;
    } else if(!ctx.hasInitialized) {
      geometry_msgs::msg::TransformStamped tmsg;
      tmsg.header.stamp = ctx.get_clock()->now();
      tmsg.transform.translation.x = 0;
      tmsg.child_frame_id = target_frame;
      tmsg.header.frame_id = map_frame;
      //ctx.tfBroad->sendTransform(tmsg);
      nav_msgs::msg::Odometry odom;
      odom.child_frame_id = target_frame;
      odom.header.frame_id = map_frame;
      odom.header.stamp = ctx.get_clock()->now();
      odom.pose.covariance = {};
      ctx.hasInitialized = true;
      ctx.odomPublisher->publish(odom);
    }

    // Filter for color of walls

    pcl::ConditionAnd<PointT>::Ptr range1(new pcl::ConditionAnd<PointT>());
    //range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("s", pcl::ComparisonOps::GT, 0)));
    range1->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("h", pcl::ComparisonOps::LT, 40)));
    range1->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("h", pcl::ComparisonOps::GT, 20)));
    range1->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("s", pcl::ComparisonOps::GT, 0.2)));
    range1->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("v", pcl::ComparisonOps::LT, 0.8)));

    pcl::ConditionAnd<PointT>::Ptr range2(new pcl::ConditionAnd<PointT>());
    //range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("s", pcl::ComparisonOps::GT, 0)));
    range2->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("h", pcl::ComparisonOps::LT, 60)));
    range2->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("h", pcl::ComparisonOps::GT, 35)));
    range2->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("s", pcl::ComparisonOps::GT, 0.2)));
    range2->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("s", pcl::ComparisonOps::LT, 0.45)));
    range2->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("v", pcl::ComparisonOps::LT, 0.8)));
    

    pcl::ConditionOr<PointT>::Ptr range_cond(new pcl::ConditionOr<PointT>());
    range_cond->addCondition(range1);
    range_cond->addCondition(range2);
    //range_cond->addComparison(pcl::FieldComparison<PointT>::Ptr(new pcl::FieldComparison<PointT>("v", pcl::ComparisonOps::LT, maxY)));

    pcl::ConditionalRemoval<PointT> range_filt;
    range_filt.setInputCloud(wrapper->transformed_cloud);
    range_filt.setCondition(range_cond);
    pcl::PointCloud<PointT>::Ptr filtered_hsv(new pcl::PointCloud<PointT>);
    range_filt.filter(*filtered_hsv);
      

    // PLANAR FITTING
    int nr_points = (int)wrapper->transformed_cloud->size();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(filtered_hsv);
    seg.setAxis({0.0f, 0.0f, 1.0f});
    seg.setEpsAngle(0.3);

    

    std::vector<std::pair<size_t, pcl::ModelCoefficients::Ptr>> coefficients_list;
    pcl::PointCloud<PointT>::Ptr filter_cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*filtered_hsv, *filter_cloud);
    pcl::PointCloud<PointT>::Ptr filter_cloud_temp(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane_cloud_temp(new pcl::PointCloud<PointT>);
    int i = 0;
    while (filter_cloud->size() > 0.1 * nr_points && i < 4)
    {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      seg.setInputCloud(filter_cloud);
      seg.segment(*inliers, *coefficients);
      if(inliers->indices.size() == 0) break;
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filter_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      if (inliers->indices.size() > 500)
      {
        coefficients_list.push_back({inliers->indices.size(), coefficients});
        extract.filter(*plane_cloud_temp);
        *plane_cloud += *plane_cloud_temp;
      }

      // Remove Points that created plane
      extract.setNegative(true);
      extract.filter(*filter_cloud_temp);
      *filter_cloud = *filter_cloud_temp;
      i++;
    }

    sensor_msgs::msg::PointCloud2 pubMsg;
    pcl::toROSMsg(*plane_cloud, pubMsg);
    pubMsg.header.frame_id = target_frame;
    ctx.publisher_->publish(pubMsg);

    //std::cout << "Found " << coefficients_list.size() << " valid planes" << std::endl;
    // FIND INTERSECTION OF PLANES
    // Find 2 biggest planes
    auto end = coefficients_list.end(); //std::remove_if(coefficients_list.begin(), coefficients_list.end(), [](auto a)
                              //{ return fabs(a.second->values[2]) > 0.3; });
    if (end - coefficients_list.begin() >= 2)
    {
      std::partial_sort(coefficients_list.begin(),
                        coefficients_list.begin() + 2, end, [](auto a, auto b)
                        { return a.first > b.first; });
      auto coeffA = coefficients_list[0];
      auto coeffB = coefficients_list[1];
      //std::cout << coeffA.first << " " << coeffB.first << std::endl;
      
      Eigen::Vector4f a(coeffA.second->values.data());
      Eigen::Vector4f b(coeffB.second->values.data());
      // Verify the orthogonality of these planes
      
      bool orthogonal = fabs(a.head<3>().dot(b.head<3>())) < 0.2;

      WallType wallA = classifyWallType(ctx, a);
      WallType wallB = classifyWallType(ctx, b);
      bool hasNorth = false, hasSouth = false, hasEast = false, hasWest = false;
      // Wall for determining x position (East and West)
      Eigen::Vector4f xWall;
      // Wall for determining y position (North and South)
      Eigen::Vector4f yWall;
      if(wallA == WallType::NORTH) {
        hasNorth = true;
        xWall = a;
      }
      if(wallB == WallType::NORTH) {
        hasNorth = true;
        xWall = b;
      }

      if(wallA == WallType::SOUTH) {
        hasSouth = true;
        xWall = a;
      }
      if(wallB == WallType::SOUTH) {
        hasSouth = true;
        xWall = b;
      }

      if(wallA == WallType::EAST) {
        hasEast = true;
        yWall = a;
      }
      if(wallB == WallType::EAST) {
        hasEast = true;
        yWall = b;
      }

      if(wallA == WallType::WEST) {
        hasWest = true;
        yWall = a;
      }
      if(wallB == WallType::WEST) {
        hasWest = true;
        yWall = b;
      }
      Eigen::Vector3d best_guess;

      //std::cout << "Walls: " << (int)wallA << " " << (int)wallB << std::endl;

      if(hasNorth && hasEast) {
        best_guess = {1.17 / 2, -2.34 / 2, 0.0};
      } else if(hasNorth && hasWest) {
        best_guess = {1.17 / 2, 2.34 / 2, 0.0};
      } else if(hasSouth && hasEast) {
        best_guess = {-1.17 / 2,  -2.34 / 2, 0.0};
      } else if(hasSouth && hasWest) {
        best_guess = {-1.17 / 2,  2.34 / 2, 0.0};
      } else {
        return;
      }


      if (orthogonal)
      {
        auto locX = xWall.head<3>()*xWall[3];
        double dX = hypot(locX.x(), locX.y());

        auto locY = yWall.head<3>()*yWall[3];
        double dY = hypot(locY.x(), locY.y());
        best_guess.y() += (hasEast?1:-1)*dY;
        best_guess.x() += (hasSouth?1:-1)*dX;
        //std::cout << best_guess << std::endl;
        // Put point at z=0
        auto point = best_guess;

        if(!ctx.tf_buffer->canTransform(target_frame, map_frame, tf2::TimePointZero)) return;
        // Find closest corner based off of previous guess
        auto guess = ctx.tf_buffer->lookupTransform(target_frame, map_frame, tf2::TimePointZero);
        Eigen::Quaterniond rot_guess(guess.transform.rotation.w,
                                     guess.transform.rotation.x,
                                     guess.transform.rotation.y,
                                     guess.transform.rotation.z);
        Eigen::Vector3d trans_guess(guess.transform.translation.x,
                                    guess.transform.translation.y,
                                    guess.transform.translation.z);
        Eigen::Vector3d pos_guess = rot_guess * point + trans_guess;
        
        // Wall A is largest, compute orientation based off of it
        float initial_angle = 0;
        switch(wallA) {
          case WallType::NORTH:
            initial_angle = 3*M_PI/2.0;
            break;
          case WallType::EAST:
            initial_angle = 0;
            break;
          case WallType::SOUTH:
            initial_angle = M_PI/2;
            break;
          case WallType::WEST:
            initial_angle = M_PI;
            break;
          case WallType::NONE:
            return;
        }
        //std::cout << "here2" << std::endl;
        // Get angle from plane
        float theta;
        theta = atan2(a[0], a[1]);
        if(theta > M_PI/2 || theta < -M_PI/2) {
          theta += M_PI;
        }
        float new_angle =  theta+initial_angle;// + M_PI/2;
        //std::cout << "Point: " << point << " " << "Theta: " << theta << "Wall A: " << (int) wallA << "Wall B: " <<(int)wallB <<  std::endl;
        Eigen::Quaterniond new_rot (Eigen::AngleAxisd(new_angle, Eigen::Vector3d{0, 0, 1}));
        if(use_seperate_transform) {
          geometry_msgs::msg::TransformStamped tmsg;
          tmsg.header.stamp = wrapper->time;
          tmsg.header.frame_id = "/mytransform";
          tmsg.child_frame_id = target_frame;
          tmsg.transform.translation.x = point.x();//-rot_point.y();
          tmsg.transform.translation.y = point.y();//rot_point.x();
          tmsg.transform.translation.z = 0.0;
          tmsg.transform.rotation.w = new_rot.w();
          tmsg.transform.rotation.x = new_rot.x();
          tmsg.transform.rotation.y = new_rot.y();
          tmsg.transform.rotation.z = new_rot.z();
          ctx.tfBroad->sendTransform(tmsg);

          geometry_msgs::msg::TransformStamped tmsg2;
          // tmsg.header.stamp = this->get_clock()->now();
          tmsg.header.frame_id = map_frame;
          tmsg.child_frame_id = "/mytransform";
          tmsg.transform.translation.x = best_guess.x();
          tmsg.transform.translation.y = best_guess.y();
          tmsg.transform.translation.z = 0.0;
          tmsg.transform.rotation.w = 1.0;
          tmsg.transform.rotation.x = 0.0;
          tmsg.transform.rotation.y = 0.0;
          tmsg.transform.rotation.z = 0.0;
          ctx.tfBroad->sendTransform(tmsg);
        } else {
          geometry_msgs::msg::TransformStamped tmsg;
          tmsg.header.stamp = wrapper->time;
          tmsg.header.frame_id = map_frame;
          tmsg.child_frame_id = target_frame;
          tmsg.transform.translation.x = best_guess.x();//-rot_point.x() + best_guess.x();
          tmsg.transform.translation.y = best_guess.y(); //-rot_point.y() + best_guess.y();
          tmsg.transform.translation.z = 0.0;
          tmsg.transform.rotation.w = new_rot.w();
          tmsg.transform.rotation.x = new_rot.x();
          tmsg.transform.rotation.y = new_rot.y();
          tmsg.transform.rotation.z = new_rot.z();
          if(publish_transform) {
            ctx.tfBroad->sendTransform(tmsg);
          }

          nav_msgs::msg::Odometry odom;
          odom.child_frame_id = target_frame;
          odom.header.frame_id = map_frame;
          odom.header.stamp = wrapper->time;
          // Assume all variable are independent from each other,
          // Therefore, covariance will be diagonal
          // Since we are assuming a 2d model, z, roll, and pitch have
          // zero covariance
          odom.pose.covariance = { 0.0004,   0,   0,   0,   0,    0,
                                   0,   0.0004,   0,   0,   0,    0,
                                   0,      0,   0,   0,   0,    0,
                                   0,      0,   0,   0,   0,    0,
                                   0,      0,   0,   0,   0,    0,
                                   0,      0,   0,   0,   0, 0.04};
          odom.pose.pose.orientation = tmsg.transform.rotation;
          if(!isnan(tmsg.transform.translation.x) && !isnan(tmsg.transform.translation.y) && !isnan(tmsg.transform.translation.z)) {
            odom.pose.pose.position.x = tmsg.transform.translation.x;
            odom.pose.pose.position.y = tmsg.transform.translation.y;
            odom.pose.pose.position.z = tmsg.transform.translation.z;
            // Verify we are in the bounds of the field
            if(fabs(tmsg.transform.translation.x) < 1.3 && fabs(tmsg.transform.translation.y) < 2.4) {
              // Velocities are not calculated, so just zero everything out
              odom.twist.covariance = {};
              ctx.odomPublisher->publish(odom);
              std::cout << "Published " << odom.header.stamp.sec << std::endl;
            }
          }
        }
      }
    }

  }
