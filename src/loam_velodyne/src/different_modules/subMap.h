//
// Created by timmy on 17-9-21.
//

#ifndef PROJECT_SUBMAP_H
#define PROJECT_SUBMAP_H

//#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/video/tracking.hpp"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,sensor_msgs::Imu> slamSyncPolicy;

class SubMap{

public:

    SubMap(ros::NodeHandle nh,
    ros::NodeHandle private_nh);

    ~SubMap() {}


    message_filters::Subscriber<sensor_msgs::PointCloud2> *featurePoints_sub;

    message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;

    message_filters::Synchronizer<slamSyncPolicy> *sync_;


    void callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
                  const sensor_msgs::ImuConstPtr& imu_msg);

private:
    PointCloud pointCloud_;
    nav_msgs::Odometry odometry_;

};
Eigen::Matrix4f NdtMatch(PointCloud lastPointCloud,
                         PointCloud curPointCloud);
Eigen::Matrix4f ICPMatch(PointCloud lastPointCloud,
                         PointCloud curPointCloud);
Eigen::Matrix4f TransformToTransformation(float x, float y, float z,
                                          float translation_x,
                                          float translation_y,
                                          float translation_z);

void AddSubmap(PointCloud Pointcloud, Eigen::Vector3d Pose);

void DetectLoop(PointCloud Pointcloud, Eigen::Vector3d Pose);


void InsertToSubmap(PointCloud pointcloud);

void imuTransHandler(sensor_msgs::PointCloud2 ImuMsg);

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);

Eigen::Vector3f PoseEstimate(Eigen::Vector3f PoseLast,
                             Eigen::Vector3f tf_ndt,
                             Eigen::Vector3f tf_imu);

//struct Pose
//{
//    double x;
//    double y;
//    double z;
//    double roll;
//    double pitch;
//    double yaw;
//};

#endif //PROJECT_SUBMAP_H
