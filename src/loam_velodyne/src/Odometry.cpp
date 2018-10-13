//
// Created by lx on 18-6-19.
//

#include "Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace LxSlam{
    void Odometry::setMea(Eigen::Vector3d mea_pose){
        measure_pose_ = mea_pose;
    }

    Eigen::Vector3d Odometry::get_XYYaw_from_odom(nav_msgs::Odometry laser_odom){
        Eigen::Vector3d pose_vector;

        pose_vector[0] = laser_odom.pose.pose.position.x;
        pose_vector[1] = laser_odom.pose.pose.position.y;
        tf::Quaternion laser_odom_orietation;
        laser_odom_orietation.w() = laser_odom.pose.pose.orientation.w;
        laser_odom_orietation.x() = laser_odom.pose.pose.orientation.x;
        laser_odom_orietation.y() = laser_odom.pose.pose.orientation.y;
        laser_odom_orietation.z() = laser_odom.pose.pose.orientation.z;

        pose_vector[3] = tf::getYaw(laser_odom_orietation);

        return pose_vector;
    }

    nav_msgs::Odometry Odometry::get_odom_from_pose(Eigen::Vector3d trans_pose){
        nav_msgs::Odometry odom_matrix;

        odom_matrix.pose.pose.position.x = trans_pose[0];
        odom_matrix.pose.pose.position.y = trans_pose[1];
        odom_matrix.pose.pose.position.z = 0;
        //TODO: Add the corresponding height to the odom message

        tf::Quaternion q_pose;
        q_pose.setRPY(0,0,trans_pose[2]);
        odom_matrix.pose.pose.orientation.x = q_pose.x();
        odom_matrix.pose.pose.orientation.y = q_pose.y();
        odom_matrix.pose.pose.orientation.z = q_pose.z();
        odom_matrix.pose.pose.orientation.w = q_pose.w();

        return odom_matrix;
    }

    void Odometry::MergeSensorData(sensor_msgs::Imu imu_data,
                                   nav_msgs::Odometry vel_data,
                                   double diff_time){
//        geometry_msgs::Pose last_pose = odometry_path_.poses.end()->pose;

        geometry_msgs::Quaternion cur_q = imu_data.orientation;
        double cur_vel = vel_data.twist.twist.linear.x;

        inc_distance = (cur_vel + last_vel_) / 2 * diff_time;

        diff_theta = tf::getYaw(cur_q) - tf::getYaw(last_q_);

        inc_distance_car << inc_distance * cos(diff_theta / 2.),
                inc_distance * sin(diff_theta / 2.), diff_theta;

        predict_pose_ = pose_ekf_ + inc_distance_car;

        last_q_ = cur_q;
        last_vel_ = cur_vel;

    }

    Eigen::Matrix4f Odometry::tf_to_global_ekf(){

        std::cout << "Begin output the EKF pose !" << std::endl;

        Eigen::Vector3f ekf_pose;

        Af << 1, 0, -inc_distance * sinf(predict_pose_[2] + diff_theta/2.0),
                0, 1,  inc_distance * cosf(predict_pose_[2] + diff_theta/2.0),
                0, 0, 1;

        Q << 2 * diff_time, 0, 0,
                0, 2 * diff_time, 0,
                0, 0, 2 * diff_time;

        R << 5, 0, 0,
                0, 5, 0,
                0, 0, 0.5;

        pred_true_Covariance = Af * esti_true_Covariance * Af.transpose() + Q;

        k_filter = pred_true_Covariance * (pred_true_Covariance + R).inverse();
        Eigen::Vector3f diff_pose;
        diff_pose = measure_pose_ - predict_pose_;
        diff_pose[2] = atan2f(sinf(diff_pose[2]), cosf(diff_pose[2]));
        ekf_pose = predict_pose_ + k_filter * diff_pose;
        float mea_yaw = measure_pose_[2];
        float pre_yaw = predict_pose_[2];
        float k_yaw = k_filter(2, 2);

        pose_ekf_[2] = atan2f(k_yaw * sinf(mea_yaw) + (1 - k_yaw) * sinf(pre_yaw),
                             k_yaw * cosf(mea_yaw) + (1 - k_yaw) * cosf(pre_yaw));
        esti_true_Covariance = (Eigen::Matrix3f::Identity() - k_filter) * pred_true_Covariance;

//        odometry_path_.poses.push_back(pose_ekf_);

        return pose_to_matrix(pose_ekf_[0], pose_ekf_[1], 0,
                              0, 0, pose_ekf_[2]);
    }

    pcl::PointCloud<pcl::PointXYZI> Odometry::filter_floor
            (sensor_msgs::PointCloud2 cloud){

    }

    void Odometry::update_path(){

    }
}