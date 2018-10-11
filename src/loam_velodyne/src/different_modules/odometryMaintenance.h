//
// Created by lx on 18-4-3.
//

#ifndef PROJECT_ODOMETRYMAINTENANCE_H
#define PROJECT_ODOMETRYMAINTENANCE_H

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "common_function.h"
#include <loam_velodyne/SegmentCloud.h>

bool get_obd = false;
bool get_imu = false;
bool get_ekf = false;
bool time_init = false;
bool vel_init = false;
bool theta_init = false;

float cur_vel, last_vel = 0;
float cur_theta_yaw, cur_theta_roll, cur_theta_pitch, last_theta = 0;
double ekf_time, cur_time, imu_time, last_time = 0;

ros::Publisher *pose_compensation_pub = NULL;
nav_msgs::Path odometry_path;
#endif //PROJECT_ODOMETRYMAINTENANCE_H
