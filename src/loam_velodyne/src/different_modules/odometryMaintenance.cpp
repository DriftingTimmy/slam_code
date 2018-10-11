//
// Created by lx on 18-4-3.
//
#include "odometryMaintenance.h"

void EKFPoseHandler(loam_velodyne::SegmentCloud ekf_pose){

    pcl::PointCloud<pcl::PointXYZ> ekf_point(2,1);
//    pcl::fromROSMsg(ekf_pose, ekf_point);

//    ekf_time = ekf_point.points[1].x;
    ekf_time = ekf_pose.time_stamp;

//    std::cout << "ekf_time is : " << std::fixed << ekf_time << " " << std::fixed << cur_time << std::endl;
    std::cout << "Speed is : " << cur_vel << std::endl;

    //TODO : figure out the problem of the time difference
    double diff_time = cur_time - ekf_time;
    double increase_dist = (cur_vel + last_vel) / 2 * diff_time;
    float diff_theta = cur_theta_yaw - ekf_point.points[0].z;
    double compensate_x = increase_dist * cosf(diff_theta / 2.);
    double compensate_y = increase_dist * sinf(diff_theta / 2.);

    ekf_pose.centroid[0] += compensate_x;
    ekf_pose.centroid[1] += compensate_y;
    ekf_pose.centroid[2] = cur_theta_yaw;
    ekf_pose.time_stamp = cur_time;

//    pcl::toROSMsg(ekf_point, ekf_pose);

    pose_compensation_pub->publish(ekf_pose);
//    std::cout << "The corresponding yaw angle : " << ekf_pose.centroid[2] << " " << cur_theta_yaw << std::endl;
    std::cout << "Pose output is : " << ekf_pose.centroid[0] << " "
              << ekf_pose.centroid[1] << " " << ekf_pose.centroid[2] << std::endl;
//    std::cout << "Diff time is : " << diff_time << std::endl;

    last_vel = cur_vel;
//    last_theta = cur_theta_yaw;

    get_ekf = true;
}

void ObdHandler(nav_msgs::Odometry vel_msg){

    if (!std::isnan(vel_msg.twist.twist.linear.x)){
        if(!vel_init){
            last_vel = vel_msg.twist.twist.linear.x;
            vel_init = true;
        } else
            cur_vel = vel_msg.twist.twist.linear.x;
    }

    cur_time = vel_msg.header.stamp.toSec();
//    std::cout << "Current time is : " << std::fixed << cur_time << std::endl;
    if(!time_init){
        ekf_time = cur_time;
        time_init = true;
    }
    get_obd = true;
}

void ImuHandler(sensor_msgs::Imu imu_msg){

    imu_time = imu_msg.header.stamp.toSec();
    geometry_msgs::Quaternion imu_quaternion = imu_msg.orientation;
    cur_theta_yaw = -tf::getYaw(imu_quaternion);

    get_imu = true;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "odometryMaintenance");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber ekf_pose_sub = nh.subscribe<loam_velodyne::SegmentCloud>("/pose_ekf", 1, EKFPoseHandler);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/nav440/nav440", 1, ImuHandler);
    ros::Subscriber obd_sub = nh.subscribe<nav_msgs::Odometry>("/canbus/canbus", 10, ObdHandler);

    ros::Publisher compensated_pose_puber = nh.advertise<loam_velodyne::SegmentCloud>("/compensated_pose", 1);
    pose_compensation_pub = &compensated_pose_puber;

    ros::spin();

    return 0;
}
