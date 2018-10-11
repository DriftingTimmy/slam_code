//
// Created by timmy on 17-11-28.
//
#include <math.h>

#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <loam_velodyne/MatchedPair.h>
#include "back_end_odom.h"
#include "subMap.h"
#include "common_function.h"
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ Point;
using namespace std;
const string path_ = "/media/lx/文件及资料/Data/KeyFrames/meituan2/";
const int pcd_num_ = 597;
g2o::SparseOptimizer odom_optimizer;
const int loop_threshold = 500;
int search_neighbour_num = 10;
float icp_score = 0.1;
ofstream posefile_aftOpt("/media/lx/文件及资料/Data/KeyFrames/meituan2/posefile_aftOpt.txt");

PointCloud trajectory_;
Eigen::Matrix4f tf_to_init_cur;
Eigen::Matrix4f tf_to_init_last;
Eigen::Matrix4f tf_between_vertex;

PointCloud ReadPCDFile(int ind){

    string file_path = path_;
    stringstream ss;
    ss << ind;
    string file_name = ss.str();
    file_name.append("laser.pcd");
    file_path.append(file_name);
    PointCloud cloud;
    pcl::io::loadPCDFile(file_path, cloud);

    return cloud;
}

void AddVertexAndEdge(int odom_counter){

    Eigen::Isometry3d vertex_transform;
    Matrix_to_Isometry(tf_to_init_cur,vertex_transform);

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(odom_counter - 1);
    v->setEstimate(vertex_transform);/// vertex pose = current pose
    odom_optimizer.addVertex(v);

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->setVertex(0, odom_optimizer.vertex(odom_counter - 2));
    edge->setVertex(1, odom_optimizer.vertex(odom_counter - 1));
    edge->setRobustKernel(new g2o::RobustKernelHuber());

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

    information(0, 0) = information(1, 1) = information(2, 2) = 50;
    information(3, 3) = information(4, 4) = information(5, 5) = 50;

    edge->setInformation(information);

    Eigen::Isometry3d T;
    Matrix_to_Isometry(tf_between_vertex,T);

    edge->setMeasurement(T);

    odom_optimizer.addEdge(edge);

}

void AddLoopEdge(int frame_id1, int frame_id2, Eigen::Matrix4f &tf_for_loop){

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->setVertex(0, odom_optimizer.vertex(frame_id1));
    edge->setVertex(1, odom_optimizer.vertex(frame_id2));
    edge->setRobustKernel(new g2o::RobustKernelHuber());

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

    information(0, 0) = information(1, 1) = information(2, 2) = 100;
    information(3, 3) = information(4, 4) = information(5, 5) = 100;

    edge->setInformation(information);

    Eigen::Isometry3d T;
    Matrix_to_Isometry(tf_for_loop,T);

    edge->setMeasurement(T);

    odom_optimizer.addEdge(edge);

}

Eigen::Matrix4f transform_to_local(std::vector<float> pose){

    float trans_x = pose.at(1);
    float trans_y = pose.at(2);
    float trans_z = pose.at(3);
    float trans_R = pose.at(4);
    float trans_P = pose.at(5);
    float trans_Y = pose.at(6);

    Eigen::Matrix4f trans_to_local = pose_to_matrix(trans_x, trans_y, trans_z,
                                                    trans_R, trans_P, trans_Y).inverse();
}

void LoopDetectionICP(int frame_id1, int frame_id2,
                   std::vector<float> pose1, std::vector<float> pose2){

    PointCloud cloud1 = ReadPCDFile(frame_id1);
    PointCloud cloud2 = ReadPCDFile(frame_id2);

//    Eigen::Matrix4f initial_guess = ComputeTransBetweenPoses(pose1, pose2);
//    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
//    Eigen::Matrix4f tf_to_local1 = transform_to_local(pose1).inverse();
//    Eigen::Matrix4f tf_to_local2 = transform_to_local(pose2).inverse();
//
//    pcl::transformPointCloud(cloud1, cloud1, tf_to_local1);
//    pcl::transformPointCloud(cloud2, cloud2, tf_to_local2);

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

    float score = icp.getFitnessScore();

    std::cout << score << std::endl;

    if( score < icp_score ){
        result = result.inverse();
        AddLoopEdge(frame_id1, frame_id2, result);
        std::cout << "Add loop edge!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

}

void LoopDetectionNDT(int frame_id1, int frame_id2,
                   std::vector<float> pose1, std::vector<float> pose2){

    PointCloud cloud1 = ReadPCDFile(frame_id1);
    PointCloud cloud2 = ReadPCDFile(frame_id2);

//    Eigen::Matrix4f initial_guess = ComputeTransBetweenPoses(pose1, pose2);
//    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
//    Eigen::Matrix4f tf_to_local1 = transform_to_local(pose1).inverse();
//    Eigen::Matrix4f tf_to_local2 = transform_to_local(pose2).inverse();
//
//    pcl::transformPointCloud(cloud1, cloud1, tf_to_local1);
//    pcl::transformPointCloud(cloud2, cloud2, tf_to_local2);

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud::Ptr lastPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud1));
    PointCloud::Ptr curPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud2));

    std::cout << "NDT is working !!!" << std::endl;
    std::cout << "Frame " << frame_id1 << " and " << frame_id2 << " is matching !" << std::endl;

    double all_start = std::clock();

    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setTransformationEpsilon(0.05);
    ndt.setStepSize(0.2);
    ndt.setResolution(0.7);
    ndt.setMaximumIterations(35);
    ndt.setInputSource(curPointCloudPtr);
    ndt.setInputTarget(lastPointCloudPtr);
    ndt.align(*output_cloud, initial_guess);

    result = ndt.getFinalTransformation();

    float score = ndt.getFitnessScore();

    double all_end = std::clock();
    std::cout << "NDT costs : " << all_end - all_start << std::endl;

    std::cout << score << std::endl;

    if( score < icp_score ){
        result = result.inverse();
        AddLoopEdge(frame_id1, frame_id2, result);
        std::cout << "Add loop edge!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

}

void UpdateMap(){

    for (size_t i = 0; i< pcd_num_; i++) {
        ///get a vertex data from g2o
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(odom_optimizer.vertex( i ));
        ///get the pose after optimizing
        Eigen::Isometry3d pose_optimize = vertex->estimate();
        ///get the corresponding original local laser data from STL of the keyframes
//        pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZI>(keyframes_[i].laser_points));

//        std::cout<<"i "<<i<<std::endl;
//        std::cout<<"vertex_pose "<< pose_optimize.matrix()<<std::endl;

        Pose v_p;
        Eigen::Matrix4f pose_matrix ;
        Isometry_to_Matrix(pose_optimize,pose_matrix);
        matrix_to_pose(pose_matrix, v_p.x, v_p.y, v_p.z, v_p.roll, v_p.pitch, v_p.yaw) ;

//        v_p.z = v_p.z * loop_z_ratio_;

        ///generate loop trajectory
        pcl::PointXYZ p_trajectory;
        p_trajectory.y = v_p.x ;
        p_trajectory.z= v_p.y ;
        p_trajectory.x = v_p.z ;
        trajectory_.push_back(p_trajectory);

        ///save the pose after optimizing
        posefile_aftOpt << i <<"    "<<v_p.x<<"  "<<v_p.y<<"  "<<v_p.z<<"  "<<
                           v_p.roll<<"   "<<v_p.pitch<<"  "<<v_p.yaw<<std::endl;

//        ///generate transform matrix according to the optimized pose
//        Eigen::Matrix4f key_pose_transform = pose_to_matrix(v_p.x,v_p.y,v_p.z,v_p.roll,v_p.pitch,v_p.yaw);
//        std::cout<<"key_pose_transform "<<key_pose_transform<<std::endl;
//
//        ///transform local laser data to global data
//        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
//        pcl::transformPointCloud(*newCloud, *tmp, key_pose_transform);
//        tmp->height=1;
//        tmp->width=tmp->size();
//        tmp->points.resize(tmp->width*tmp->height);
//
//        ///save global single frame laser data
//        std::string global_path = cloud_path(save_loop_global_lidar_path_, keyframes_[i].key_frame_id, "global.pcd");
//        pcl::io::savePCDFileBinary(global_path, *tmp );
//
//        tmp->clear() ;
//        newCloud->clear() ;
    }
    pcl::io::savePCDFileBinary( "/media/lx/文件及资料/Data/KeyFrames/meituan2/trajectory_ndt.pcd", trajectory_ );
    std::cout << "Trajectory has been saved! " << std::endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "LoopOptimizing");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    fstream pose_file("/media/lx/文件及资料/Data/KeyFrames/meituan2/posefile.txt");
    static int counter = 0;
    float x, y, z, roll, pitch, yaw;
    int frame_id;
    std::vector<std::vector<float>> pose_data;
    pcl::PointCloud<pcl::PointXYZ> pose_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    ///Add vertex and edges between vertexes while reading the
    ///corresponding pose data file and push the data into RAM
    for (int counter = 1; counter <= pcd_num_ ; ++counter) {

        std::vector<float> data = get_txt_data(pose_file, counter, 7);
        frame_id = data.at(0);

        tf_to_init_cur = pose_to_matrix(data.at(1), data.at(2), data.at(3),
                                        data.at(4), data.at(5), data.at(6));
        if (counter == 1){

            SlamLinearSolver* linearSolver = new SlamLinearSolver();
            linearSolver->setBlockOrdering( false );
            SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
            g2o::OptimizationAlgorithmLevenberg* solver =
                    new g2o::OptimizationAlgorithmLevenberg( blockSolver );
            odom_optimizer.setAlgorithm( solver );

            odom_optimizer.setVerbose( false );

            g2o::VertexSE3* v = new g2o::VertexSE3();
            v->setId(0);
            v->setEstimate( Eigen::Isometry3d::Identity() ); ///估计为单位矩阵
            v->setFixed( true ); ///第一个顶点固定，不用优化
            odom_optimizer.addVertex( v );
        } else{
            tf_between_vertex = tf_to_init_last.inverse() * tf_to_init_cur;
            AddVertexAndEdge(counter);
        }

        tf_to_init_last = tf_to_init_cur;
        pose_data.push_back(data);
        std::cout << data.at(0) << " " << data.at(1) << " " << data.at(2) << " "
                  << data.at(3) << " " << data.at(4) << " "
                << data.at(5) << " " << data.at(6) << std::endl;
    }

    std::cout << pose_data.size() << std::endl;

    for (int i = 0; i < pose_data.size() ; ++i) {

        pcl::PointXYZ pose_point;
        pose_point.x = pose_data.at(i).at(1);
        pose_point.y = pose_data.at(i).at(2);
        pose_point.z = pose_data.at(i).at(3);
//        std::cout << i << std::endl;
        pose_cloud.push_back(pose_point);

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pose_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(pose_cloud));
    std::cout << "fuck it double " << std::endl;

    kdtree.setInputCloud(pose_cloud_ptr);

    for (int counter = loop_threshold; counter < pcd_num_  ; ++counter) {

        pcl::PointXYZ searchPoint;
        searchPoint.x = pose_data.at(counter).at(1);
        searchPoint.y = pose_data.at(counter).at(2);
        searchPoint.z = pose_data.at(counter).at(3);

        std::vector<int> pointIdxNKNSearch(search_neighbour_num);
        std::vector<float> pointNKNSquaredDistance(search_neighbour_num);

        if ( kdtree.nearestKSearch (searchPoint, search_neighbour_num,
                                    pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            //std::cout << "Kdtree is working !!!!" << std::endl;
            for (unsigned int i = 0; i < pointIdxNKNSearch.size (); ++i){

                if(pointNKNSquaredDistance[i] < 10 &&
                        fabs(counter - pointIdxNKNSearch[i]) > 300)
                    LoopDetectionNDT( pointIdxNKNSearch.at(i), counter,
                                   pose_data.at(pointIdxNKNSearch.at(i)), pose_data.at(counter));
            }
        }

        if ( counter == pcd_num_ - 1){
//            odom_optimizer.save("/media/lx/文件及资料/Data/KeyFrames/meituan2/result_before.g2o");
//
            Eigen::Matrix4f test_tf = Eigen::Matrix4f::Identity();
            AddLoopEdge(28, 595, test_tf);

            odom_optimizer.initializeOptimization();
            odom_optimizer.optimize(50);
//
//            odom_optimizer.save( "/media/lx/文件及资料/Data/KeyFrames/meituan2/result_after.g2o" );
//            std::cout << "result has been saved" << std::endl;
//
            UpdateMap();
        }
    }

    return 0;
}
