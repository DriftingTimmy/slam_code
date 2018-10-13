//
// Created by lx on 18-6-19.
//

#ifndef PROJECT_ODOMETRY_H
#define PROJECT_ODOMETRY_H

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <sensor_msgs/PointCloud2.h>
#include <common_function.h>

namespace LxSlam{

class Odometry {
public:
    void MergeSensorData(sensor_msgs::Imu imu_data,
                         nav_msgs::Odometry vel_data,
                         double diff_time){};

    Eigen::Matrix4f get_ekf_pose(){
        tf_to_global_ekf();
        return pose_ekf_;};

    Eigen::Vector3d get_XYYaw_from_odom(nav_msgs::Odometry laser_odom){};

    nav_msgs::Odometry get_odom_from_pose(Eigen::Vector3d trans_pose){};

    void setMea(Eigen::Vector3d mea_pose){};

//    void set_tf_pre(Eigen::Matrix4f tf_pre){ tf_prediction_ = tf_pre; };
//    void set_tf_mea(Eigen::Matrix4f tf_mea) {
//        tf_measurement_ = tf_mea; };//set loam output as the measurement tf

private:

    Eigen::Matrix4f tf_to_global_ekf(){};
    void update_path(){};
    void add_pose_to_path(){};

    Eigen::Vector3d pose_ekf_;
    Eigen::Vector3d predict_pose_;
    Eigen::Vector3d measure_pose_;
    nav_msgs::Path  odometry_path_;

    float last_vel_;
    geometry_msgs::Quaternion last_q_;
    Eigen::Vector3d inc_distance_car;
    double inc_distance;
    double diff_time;
    double diff_theta;

    Eigen::Matrix3f Af;
    Eigen::Matrix3f Q;
    Eigen::Matrix3f R;                      //一次卡尔曼测量协方差矩阵
    Eigen::Matrix3f pred_true_Covariance;   //一次卡尔曼预测值与真实值之间的协方差矩阵
    Eigen::Matrix3f esti_true_Covariance;   // 1次卡尔曼测量值与真实值之前的协方差矩阵
    Eigen::Matrix3f k_filter;               //一次卡尔曼系数矩阵
    ///EKF Matrix params
};

}///namespace LxSlam

#endif //PROJECT_ODOMETRY_H
