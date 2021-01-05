/******************************************************************************
 * Copyright 2020 The CIDI Authors. All Rights Reserved.
 * @author: cuiDarchan
 *****************************************************************************/

// C++
#include <math.h>
#include <omp.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "Eigen/Dense"

// msgs
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/UInt32.h"

// pcl
#include <pcl/common/transforms.h>  //	pcl::transformPointCloud 用到这个头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// yaml
#include "yaml-cpp/yaml.h"

struct PointXYZIRT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  uint8_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(double, timestamp, timestamp))

typedef PointXYZIRT Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;

// 文件参数
ros::Subscriber sub_livox_compensator_;
ros::Publisher pub_livox_car_;

void callback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud) {
  PointCloudPtr pc(new PointCloud);
  pcl::fromROSMsg(*point_cloud, *pc);

  double time_base = ros::Time::now().toSec();  // 加载开始
  Eigen::Matrix4d imu_to_car;
  imu_to_car << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  pcl::transformPointCloud(*pc, *pc, imu_to_car);

  sensor_msgs::PointCloud2 pub_msg;
  pcl::toROSMsg(*pc, pub_msg);
  pub_msg.header.stamp = ros::Time::now();
  pub_msg.header.frame_id = "car";
  pub_livox_car_.publish(pub_msg);  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu2car");
  ros::NodeHandle nh("~");

  sub_livox_compensator_ = nh.subscribe<sensor_msgs::PointCloud2>(
      "/livox_lidar_front/compensator/PointCloud2", 10, &callback);
  pub_livox_car_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/livox_lidar_front/car/PointCloud2", 10);  
  ros::spin();
  return 0;
}