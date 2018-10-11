//
// Created by lx on 17-12-22.
//

#include <math.h>
#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <loam_velodyne/KeyFrame.h>
#include <loam_velodyne/MatchedPair.h>
#include "subMap.h"
#include "common_function.h"
#include "tinyxml.h"

using namespace std;

const int search_neighbour_num = 15;
const float icp_score = 2;
int frame_id;

//struct ScorePair{
//    int id1;
//    int id2;
//    float icp_score;
//};

pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::PointCloud<pcl::PointXYZ> pose_cloud;

ros::Publisher pubConnectedFrames;
ros::Publisher pubGridSearchFrames;

//TODO: Make the score from high to low and select the best loop match
//std::vector<Pose> PosesForLoop;
//std::vector<ScorePair> icp_set;

std::string read_config_path;
std::string KeyFrame_save_path;

void read_xml(){

    TiXmlDocument doc ;
    if(!doc.LoadFile(read_config_path))
    {
        std::cout<<"error_xmllasermapping"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = doc.FirstChildElement("lx_mapping") ;

    TiXmlElement* KeyFrame_save_path_Elem         = node->FirstChildElement("KeyFrame_save_path") ;

    KeyFrame_save_path  = KeyFrame_save_path_Elem->GetText();
}

PointCloud ReadPCDFile(int ind){

    string file_path = KeyFrame_save_path;
    stringstream ss;
    ss << ind;
    string file_name = ss.str();
    file_name.append("laser.pcd");
    file_path.append(file_name);
    PointCloud cloud;
    pcl::io::loadPCDFile(file_path, cloud);

    return cloud;
}

void LoopDetectionICP(int frame_id1, int frame_id2){

    PointCloud cloud1 = ReadPCDFile(frame_id1);
    PointCloud cloud2 = ReadPCDFile(frame_id2);

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud::Ptr lastPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud1));
    PointCloud::Ptr curPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud2));

    std::cout << "ICP is working !!!" << std::endl;
    std::cout << "Frame " << frame_id1 << " and " << frame_id2 << " is matching !" << std::endl;

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setEuclideanFitnessEpsilon(0.05);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations(100);
    icp.setInputSource(curPointCloudPtr);
    icp.setInputTarget(lastPointCloudPtr);

    icp.align(*output_cloud, initial_guess);

    result = icp.getFinalTransformation();

    double score = icp.getFitnessScore();

    std::cout << "ICP score is : " << score << std::endl;

    if( score < icp_score ){
        loam_velodyne::MatchedPair pair_to_add_edge;
        double trans[6];
        matrix_to_pose(result, trans[0], trans[1], trans[2], trans[3], trans[4], trans[5]);

        pair_to_add_edge.transformation.at(0) = trans[0];
        pair_to_add_edge.transformation.at(1) = trans[1];
        pair_to_add_edge.transformation.at(2) = trans[2];
        pair_to_add_edge.transformation.at(3) = trans[3];
        pair_to_add_edge.transformation.at(4) = trans[4];
        pair_to_add_edge.transformation.at(5) = trans[5];

        pair_to_add_edge.id1 = frame_id1;
        pair_to_add_edge.id2 = frame_id2;

        pubConnectedFrames.publish(pair_to_add_edge);

        std::cout << "Publish loop edge!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

}


//void LoopDetectionNDT(int frame_id1, int frame_id2){
//
//    PointCloud cloud1 = ReadPCDFile(frame_id1);
//    PointCloud cloud2 = ReadPCDFile(frame_id2);
//
//    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
//
//    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
//    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
//    PointCloud::Ptr lastPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud1));
//    PointCloud::Ptr curPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud2));
//
//    std::cout << "NDT is working !!!" << std::endl;
//    std::cout << "Frame " << frame_id1 << " and " << frame_id2
//              << " is matching !" << std::endl;
//
//    double all_start = std::clock();
//
//    pcl::NormalDistributionsTransform<Point, Point> ndt;
//    ndt.setTransformationEpsilon(0.05);
//    ndt.setStepSize(0.2);
//    ndt.setResolution(0.7);
//    ndt.setMaximumIterations(35);
//    ndt.setInputSource(curPointCloudPtr);
//    ndt.setInputTarget(lastPointCloudPtr);
//    ndt.align(*output_cloud, initial_guess);
//
//    result = ndt.getFinalTransformation();
//
//    float score = ndt.getFitnessScore();
//
//    double all_end = std::clock();
//    std::cout << "NDT costs : " << all_end - all_start << std::endl;
//
//    std::cout << score << std::endl;
//
//    if( score < icp_score ){
////        result = result.inverse();
//        loam_velodyne::MatchedPair pair_to_add_edge;
//        double trans[6];
//        matrix_to_pose(result, trans[0], trans[1], trans[2], trans[3], trans[4], trans[5]);
//        pair_to_add_edge.transformation.at(0) = trans[0];
//        pair_to_add_edge.transformation.at(1) = trans[1];
//        pair_to_add_edge.transformation.at(2) = trans[2];
//        pair_to_add_edge.transformation.at(3) = trans[3];
//        pair_to_add_edge.transformation.at(4) = trans[4];
//        pair_to_add_edge.transformation.at(5) = trans[5];
//
//        pair_to_add_edge.id1 = frame_id1;
//        pair_to_add_edge.id2 = frame_id2;
//
//        pubConnectedFrames.publish(pair_to_add_edge);
//
//        std::cout << "Publish loop edge!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//    }
//
//}

void SearchNearbyPose(pcl::PointXYZ point_to_detect){

    std::vector<int> pointIdxNKNSearch(search_neighbour_num);
    std::vector<float> pointNKNSquaredDistance(search_neighbour_num);

    if ( kdtree.nearestKSearch (point_to_detect, search_neighbour_num,
                                pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        for (unsigned int i = 0; i < pointIdxNKNSearch.size (); ++i){
//TODO: Find a way to fix the kdtree like pointcloud;
            loam_velodyne::MatchedPair pair;
            pair.id1 = frame_id;
            pair.id2 = pointIdxNKNSearch.at(i);

            if(frame_id - pointIdxNKNSearch.at(i) > 200){

                std::cout << frame_id << "    " << pointIdxNKNSearch.at(i) << std::endl;
                pubGridSearchFrames.publish(pair);

            }
        }
    }
}

void KeyFrameHandler(loam_velodyne::KeyFrame keyframe){

    pcl::PointXYZ pose_point;
    pose_point.x = keyframe.x;
    pose_point.y = keyframe.y;
    pose_point.z = keyframe.z;
    frame_id = keyframe.frame_id;
    pose_cloud.push_back(pose_point);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
            pose_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(pose_cloud));

    kdtree.setInputCloud(pose_cloud_ptr);
    if(frame_id > 100){

        SearchNearbyPose(pose_point);
    }
}

void PairHandler(loam_velodyne::MatchedPair pair){

    int ind1 = pair.id1;
    int ind2 = pair.id2;

    LoopDetectionICP(ind1, ind2);
}

//TODO: Match with the submap rather than only one frame in history
int main(int argc, char** argv){

    ros::init(argc, argv, "looping");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("read_config_path",read_config_path);
    read_xml();

    ros::Subscriber odom_sub_from_loam = nh.subscribe<loam_velodyne::KeyFrame>
            ("/Key_Frame", 2, KeyFrameHandler);

    ros::Subscriber Matched_grid_sub = nh.subscribe<loam_velodyne::MatchedPair>
            ("/Matched_grid", 2, PairHandler);

    pubConnectedFrames = nh.advertise<loam_velodyne::MatchedPair>("/ConnectedFrames", 2);
    pubGridSearchFrames = nh.advertise<loam_velodyne::MatchedPair>("/Grid_to_search", 2);

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}