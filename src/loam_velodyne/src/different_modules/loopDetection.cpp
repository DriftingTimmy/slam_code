//
// Created by timmy on 17-11-15.
//
#include <math.h>

#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <loam_velodyne/KeyFrame.h>
#include <loam_velodyne/MatchedPair.h>
#include <pcl/registration/ndt.h>
#include "back_end_odom.h"
#include "subMap.h"
#include "common_function.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ Point;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

//struct Pose{
//    int id;
//    float x;
//    float y;
//    float z;
//    float roll;
//    float pitch;
//    float yaw;
//};

std::vector<Pose> PosesForLoop;

static int frame_id, frame_id_last;
Pose keyFrame_pose = {0};
Pose keyFrame_pose_last = {0};
static bool keyframe_inited = false;
static bool match_done = true;
float match_threshold = 0.5;

const int loop_threshold_num = 500;

PointCloud keyFrame_cloud;//current keyframe
PointCloud keyFrame_cloud_last;//last keyframe

ros::Publisher pubConnectedFrames;

float ComputeDistance(Pose pose1, Pose pose2){

    float diffX = pose1.x - pose2.x;
    float diffY = pose1.y - pose2.y;
    float diffZ = pose1.z - pose2.z;
    float dist = sqrtf(diffX * diffX + diffY * diffY + diffZ * diffZ);

    return  dist;
}

Eigen::Matrix4f ComputeTransformBetweenPoses(Pose pose1, Pose pose2){

    float diff_roll = pose1.roll - pose2.roll;
    float diff_pitch = pose1.pitch - pose2.pitch;
    float diff_yaw = pose1.yaw - pose2.yaw;
    float diff_x = pose1.x - pose2.x;
    float diff_y = pose1.y - pose2.y;
    float diff_z = pose1.z - pose2.z;

    Eigen::Matrix4f transform = TransformToTransformation
            (diff_roll, diff_pitch, diff_yaw,
             diff_x, diff_y, diff_z);
    return transform;
}

PointCloud ReadPCDFile(int ind){

    string file_path = "/media/timmy/文件及资料/Data/KeyFrames/";
    stringstream ss;
    ss << ind;
    string file_name = ss.str();
    file_name.append("laser.pcd");
    file_path.append(file_name);
    PointCloud cloud;
    pcl::io::loadPCDFile(file_path, cloud);

    return cloud;
}

Pose TransformToRPY(Eigen::Matrix4f trans){
    Pose pose;
    tf::Matrix3x3 mat_rotate;
    mat_rotate.setValue(static_cast<double>(trans(0, 0)), static_cast<double>(trans(0, 1)),
                        static_cast<double>(trans(0, 2)), static_cast<double>(trans(1, 0)),
                        static_cast<double>(trans(1, 1)), static_cast<double>(trans(1, 2)),
                        static_cast<double>(trans(2, 0)), static_cast<double>(trans(2, 1)),
                        static_cast<double>(trans(2, 2)));

    double roll,pitch,yaw;
    mat_rotate.getRPY(roll, pitch, yaw);
    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
    pose.x = trans(0,3);
    pose.y = trans(1,3);
    pose.z = trans(2,3);

    return pose;

}
///NDT method to match the frames
//void MatchKeyFrame(PointCloud cloud_1, int id1,
//                   PointCloud cloud_2, int id2,
//                   Eigen::Matrix4f initial_guess_imu){
//    match_done = false;
//
//    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
//    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
//    PointCloud::Ptr lastPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud_1));
//    PointCloud::Ptr curPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud_2));
//
//    pcl::NormalDistributionsTransform<Point, Point> ndt;
//    ndt.setTransformationEpsilon(0.08);
//    ndt.setStepSize(0.2);
//    ndt.setResolution(0.7);
//    ndt.setMaximumIterations(35);
//    ndt.setInputSource(curPointCloudPtr);
//    ndt.setInputTarget(lastPointCloudPtr);
//
//    std::cout << "ndt is coming !" << std::endl;
//
//    ndt.align(*output_cloud, initial_guess_imu);
//
//    result = ndt.getFinalTransformation();
//
//    std::cout << "ndt is done ! " << std::endl;
//    float score = ndt.getFitnessScore();
//    std::cout << "Ndt score: " << score << std::endl;
//
//    ///If the frames are matched, then we send the corresponding ids to the next node;
//    if(score <= match_threshold){
//
//        loam_velodyne::MatchedPair msg;
//        msg.id1 = id1;
//        msg.id2 = id2;
//        Pose result_trans = TransformToRPY(result);
//        msg.transformation[0] = result_trans.x;
//        msg.transformation[1] = result_trans.y;
//        msg.transformation[2] = result_trans.z;
//        msg.transformation[3] = result_trans.roll;
//        msg.transformation[4] = result_trans.pitch;
//        msg.transformation[5] = result_trans.yaw;
//
//        pubConnectedFrames.publish(msg);
//
//        std::cout << "Matched KeyFrames are detected!!" << std::endl;
//    }
//
//    match_done = true;
//}

///ICP method to match the frames
void MatchKeyFrame(PointCloud cloud_1, int id1,
                   PointCloud cloud_2, int id2,
                   Eigen::Matrix4f initial_guess_imu){
    match_done = false;

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud::Ptr lastPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud_1));
    PointCloud::Ptr curPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud_2));

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setEuclideanFitnessEpsilon(0.05);
    icp.setMaxCorrespondenceDistance(0.8);
    icp.setMaximumIterations(40);
    icp.setInputSource(curPointCloudPtr);
    icp.setInputTarget(lastPointCloudPtr);

    icp.align(*output_cloud, initial_guess_imu);

    result = icp.getFinalTransformation();

    float score = icp.getFitnessScore();
    std::cout << "ICP score: " << score << std::endl;

    ///If the frames are matched, then we send the corresponding ids to the next node;
    if(score <= match_threshold){

        loam_velodyne::MatchedPair msg;
        msg.id1 = id1;
        msg.id2 = id2;
        Pose result_trans = TransformToRPY(result);
        msg.transformation[0] = result_trans.x;
        msg.transformation[1] = result_trans.y;
        msg.transformation[2] = result_trans.z;
        msg.transformation[3] = result_trans.roll;
        msg.transformation[4] = result_trans.pitch;
        msg.transformation[5] = result_trans.yaw;

        pubConnectedFrames.publish(msg);

        std::cout << "Matched KeyFrames are detected!!" << std::endl;
    }

    match_done = true;
}

void SearchNeighborPose(){
//    std::cout << "we are here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    int pose_count = PosesForLoop.size();
    for(int i = 0; i < pose_count; i++){
//        std::cout << "6666666666666666666" << std::endl;
        float pose_dist = ComputeDistance(keyFrame_pose, PosesForLoop.at(i));
        if(pose_dist < 5){
//            std::cout << "7777777777777777777777777777" << std::endl;
            ///Send the connected frame and ids
            ///Match the frames that are closed enough;
            PointCloud pointcloud_detect = ReadPCDFile(PosesForLoop.at(i).id);
//            std::cout << "7777777777777777777777777777" << std::endl;
            Eigen::Matrix4f transform = ComputeTransformBetweenPoses(keyFrame_pose,
                                                                     PosesForLoop.at(i));
//            std::cout << "888888888888888888888888888888888" << std::endl;
            MatchKeyFrame(keyFrame_cloud, frame_id,
                          pointcloud_detect, PosesForLoop.at(i).id,
                          transform);
        }
    }
}

void KeyFrameHandler(loam_velodyne::KeyFrame keyFrame){

    ///Initialize the keyframe process
    if(!keyframe_inited){
        keyframe_inited = true;
        pcl::fromROSMsg(keyFrame.pointcloud, keyFrame_cloud_last);
        keyFrame_pose_last.roll = keyFrame.roll;
        keyFrame_pose_last.pitch = keyFrame.pitch;
        keyFrame_pose_last.yaw = keyFrame.yaw;
        keyFrame_pose_last.x = keyFrame.x;
        keyFrame_pose_last.y = keyFrame.y;
        keyFrame_pose_last.z = keyFrame.z;
        frame_id_last = keyFrame.frame_id;
        keyFrame_pose_last.id = frame_id_last;
        PosesForLoop.push_back(keyFrame_pose_last);

    }
    ///Choose when to match the frame or build a new frame to be
    ///matched and search the nearby pose for loop detection
    else{

        pcl::fromROSMsg(keyFrame.pointcloud, keyFrame_cloud);
        keyFrame_pose.roll = keyFrame.roll;
        keyFrame_pose.pitch = keyFrame.pitch;
        keyFrame_pose.yaw = keyFrame.yaw;
        keyFrame_pose.x = keyFrame.x;
        keyFrame_pose.y = keyFrame.y;
        keyFrame_pose.z = keyFrame.z;
        frame_id = keyFrame.frame_id;
        keyFrame_pose.id = frame_id;
        PosesForLoop.push_back(keyFrame_pose);

//        SaveKeyFramesToPCD(keyFrame_cloud, frame_id);
        int diff_id = frame_id - frame_id_last;
        float diff_trans = ComputeDistance(keyFrame_pose, keyFrame_pose_last);

        ///If the distance and the ids are suitable to match, then we match the keyframes
        ///and search the nearby pose to find loop closure
        if(diff_id < 10 && diff_trans < 3.5){
            ///When the map contains more than 1000 scans, we try to search the
            ///near pose to detect the large loop;
            if(frame_id > loop_threshold_num){
                SearchNeighborPose();
            }
            if(match_done){
                Eigen::Matrix4f transform =
                        ComputeTransformBetweenPoses(keyFrame_pose,
                                                     keyFrame_pose_last);
                MatchKeyFrame(keyFrame_cloud, frame_id,
                              keyFrame_cloud_last, frame_id_last,
                              transform);
                keyFrame_pose_last = keyFrame_pose;
                keyFrame_cloud_last = keyFrame_cloud;
                keyFrame_cloud.clear();
                frame_id_last = frame_id;
            }

        }
        else{
            keyFrame_pose_last = keyFrame_pose;
            keyFrame_cloud_last = keyFrame_cloud;
            keyFrame_cloud.clear();
            frame_id_last = frame_id;
        }
    }

}

int main(int argc, char** argv){

    ros::init(argc, argv, "loopDetection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber odom_sub_from_loam = nh.subscribe<loam_velodyne::KeyFrame>
            ("/Key_Frame", 2, KeyFrameHandler);

    pubConnectedFrames = nh.advertise<loam_velodyne::MatchedPair>("/ConnectedFrames", 2);

    while(ros::ok()){
            ros::spinOnce();
    }

    return 0;
}