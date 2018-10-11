//
// Created by timmy on 17-11-11.
//
#include "tinyxml.h"

#include "back_end_odom.h"
#include "subMap.h"
#include "common_function.h"
#include <loam_velodyne/MatchedPair.h>

static int odom_counter = 0;
//const int run_opt_step = 60;

//double odom_from_loam_time;
float min_trans_threshold = 0.05;

float loam_tf_to_init_cur[6] = {0};
float loam_tf_to_init_last[6] = {0};
Eigen::Matrix4f tf_to_init_cur = Eigen::Matrix4f::Identity();
Eigen::Matrix4f tf_to_init_last = Eigen::Matrix4f::Identity();
Eigen::Matrix4f tf_between_vertex = Eigen::Matrix4f::Identity();

bool odom_updated_flag = false;

g2o::SparseOptimizer odom_optimizer;
std::ofstream posefile_aftOpt;
pcl::PointCloud<pcl::PointXYZ> trajectory_;

std::string read_config_path;
std::string g2o_file_path;
std::string trajectory_aft_opt;
std::string pose_aft_opt;
std::ofstream pose_g2o;
//pcl::PointCloud<pcl::PointXYZ> trajectory_before_opt;
//ros::Publisher odompuber;
//std::vector<Pose> odom_aft_opt;

void read_xml(){

    TiXmlDocument doc ;
    if(!doc.LoadFile(read_config_path))
    {
        std::cout<<"error_xmlbackend"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = doc.FirstChildElement("lx_mapping") ;

    TiXmlElement* g2o_file_path_Elem       = node->FirstChildElement("grid_map_feature_num");

    g2o_file_path    = g2o_file_path_Elem->GetText();

    trajectory_aft_opt = g2o_file_path;
    pose_aft_opt       = g2o_file_path;

    trajectory_aft_opt.append("trajectory_aft_opt.pcd");
    pose_aft_opt.append("pose_aft_opt.g2o");
    pose_g2o << pose_aft_opt;
}

void AddVertexAndEdge(){

    Eigen::Isometry3d vertex_transform;
    Matrix_to_Isometry(tf_to_init_cur,vertex_transform);

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(odom_counter);
    v->setEstimate(vertex_transform);/// vertex pose = current pose
    odom_optimizer.addVertex(v);

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->setVertex(0, odom_optimizer.vertex(odom_counter - 1));
    edge->setVertex(1, odom_optimizer.vertex(odom_counter));
    edge->setRobustKernel(new g2o::RobustKernelHuber());

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

    information(0, 0) = information(1, 1) = information(2, 2) = 100;
    information(3, 3) = information(4, 4) = information(5, 5) = 100;

    edge->setInformation(information);

    Eigen::Isometry3d T;
    Matrix_to_Isometry(tf_between_vertex,T);

    edge->setMeasurement(T);

    odom_optimizer.addEdge(edge);
}

//void AddEdgeToInit(){
//
//    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
//
//    edge->setVertex(0, odom_optimizer.vertex(0));
//    edge->setVertex(1, odom_optimizer.vertex(odom_counter));
//    edge->setRobustKernel(new g2o::RobustKernelHuber());
//
//    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
//
//    information(0, 0) = information(1, 1) = information(2, 2) = 100;
//    information(3, 3) = information(4, 4) = information(5, 5) = 100;
//
//    edge->setInformation(information);
//
//    Eigen::Isometry3d T;
//    Matrix_to_Isometry(tf_to_init_cur,T);
//
//    edge->setMeasurement(T);
//
//    odom_optimizer.addEdge(edge);
//
//}

///Add corresponding edges in the graph when the matched pairs are detected
void AddEdgeForLoop(int frameid_1, int frameid_2,
                    Eigen::Matrix4f transform){

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->setVertex(0, odom_optimizer.vertex(frameid_1));
    edge->setVertex(1, odom_optimizer.vertex(frameid_2));
    edge->setRobustKernel(new g2o::RobustKernelHuber());

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

    information(0, 0) = information(1, 1) = information(2, 2) = 100;
    information(3, 3) = information(4, 4) = information(5, 5) = 100;

    edge->setInformation(information);

    Eigen::Isometry3d T;
    Matrix_to_Isometry(transform, T);

    edge->setMeasurement(T);

    odom_optimizer.addEdge(edge);

}

float Check_Translation(){

    float diffX = loam_tf_to_init_cur[3] - loam_tf_to_init_last[3];
    float diffY = loam_tf_to_init_cur[4] - loam_tf_to_init_last[4];
    float diffZ = loam_tf_to_init_cur[5] - loam_tf_to_init_last[5];

    float dist = sqrtf(diffX * diffX + diffY * diffY +diffZ * diffZ);

    return dist;
}

void UpdateMap(){

    for (size_t i = 0; i< odom_counter; i++) {
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
    pcl::io::savePCDFileBinary( trajectory_aft_opt, trajectory_ );
    std::cout << "Trajectory has been saved! " << std::endl;
}

void OdomHandler(const nav_msgs::Odometry::ConstPtr& odom_from_loam_mapping){

//    std::cout << "Received Odom Messages!" << std::endl;

//    odom_from_loam_time = odom_from_loam_mapping->header.stamp.toSec();

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom_from_loam_mapping->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
            .getRPY(roll, pitch, yaw);

    loam_tf_to_init_cur[0] = -pitch;
    loam_tf_to_init_cur[1] = -yaw;
    loam_tf_to_init_cur[2] = roll;

    loam_tf_to_init_cur[3] = odom_from_loam_mapping->pose.pose.position.x;
    loam_tf_to_init_cur[4] = odom_from_loam_mapping->pose.pose.position.y;
    loam_tf_to_init_cur[5] = odom_from_loam_mapping->pose.pose.position.z;

    tf_to_init_cur = pose_to_matrix(loam_tf_to_init_cur[3], loam_tf_to_init_cur[4],
                                    loam_tf_to_init_cur[5], loam_tf_to_init_cur[2],
                                    loam_tf_to_init_cur[0], loam_tf_to_init_cur[1]);
//    tf_to_init_cur = pose_to_matrix(loam_tf_to_init_cur[3], loam_tf_to_init_cur[4],
//                                    loam_tf_to_init_cur[5], 0,
//                                    0, loam_tf_to_init_cur[1]);

//    pcl::PointXYZ point_odom;
//    point_odom.x = loam_tf_to_init_cur[3];
//    point_odom.y = loam_tf_to_init_cur[4];
//    point_odom.z = loam_tf_to_init_cur[5];
//    trajectory_before_opt.push_back(point_odom);
//    sensor_msgs::PointCloud2 trajectory_msg_opt;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory_before_opt_ptr
//            (new pcl::PointCloud<pcl::PointXYZ>(trajectory_before_opt));
//    pcl::toROSMsg(*trajectory_before_opt_ptr, trajectory_msg_opt);
//    trajectory_msg_opt.header.frame_id = odom_from_loam_mapping->header.frame_id;
//    trajectory_msg_opt.header.stamp = odom_from_loam_mapping->header.stamp;
//    odompuber.publish(trajectory_msg_opt);

    odom_updated_flag = true;
    odom_counter++;
}

void PairHandler(loam_velodyne::MatchedPair pair){

    float x     = pair.transformation[0];
    float y     = pair.transformation[1];
    float z     = pair.transformation[2];
    float roll  = pair.transformation[3];
    float pitch = pair.transformation[4];
    float yaw   = pair.transformation[5];

    Eigen::Matrix4f trans = pose_to_matrix(x, y, z, roll, pitch, yaw);

    AddEdgeForLoop(pair.id1, pair.id2, trans);
    std::cout << "Edges for loop are put in the graph!" << std::endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "back_end_odom");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("read_config_path", read_config_path);
    read_xml();

    ros::Subscriber odom_sub_from_loam = nh.subscribe<nav_msgs::Odometry>
            ("/Pose_For_Loop", 1, OdomHandler);
    ros::Subscriber sub_conneted_pair = nh.subscribe<loam_velodyne::MatchedPair>
            ("/ConnectedFrames", 1, PairHandler);

//    odompuber = nh.advertise<sensor_msgs::PointCloud2>("/test_odom", 2);

    /**************************Initialize G2O*********************************/
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg( blockSolver );
    odom_optimizer.setAlgorithm( solver );
    ///no debug information out
    odom_optimizer.setVerbose( false );
    //optimize_step_ = save_num_ ;

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( 0 );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    odom_optimizer.addVertex( v );

    while(ros::ok()){
        ros::spinOnce();

        ///Add vertex and edge if odom has been updated
        if(odom_updated_flag){

            //TODO: 对第一帧的数据进行处理保证添加的定点和约束序列正确且有数据
            if(odom_counter >= 1){

                float diff_trans = Check_Translation();
                std::cout << "distance is : " << diff_trans << std::endl;

                if(diff_trans >= min_trans_threshold){

//                    tf_between_vertex = tf_to_init_last * tf_to_init_cur.inverse();
                    tf_between_vertex = tf_to_init_last.inverse() * tf_to_init_cur;

                    loam_tf_to_init_last[0] = loam_tf_to_init_cur[0];
                    loam_tf_to_init_last[1] = loam_tf_to_init_cur[1];
                    loam_tf_to_init_last[2] = loam_tf_to_init_cur[2];
                    loam_tf_to_init_last[3] = loam_tf_to_init_cur[3];
                    loam_tf_to_init_last[4] = loam_tf_to_init_cur[4];
                    loam_tf_to_init_last[5] = loam_tf_to_init_cur[5];

                    AddVertexAndEdge();

                    tf_to_init_last = tf_to_init_cur;
                }
                else{
                    odom_counter--;
                    std::cout << "Translation is too small" << std::endl;
                }
            }
        }
        odom_updated_flag = false;
    }

    odom_optimizer.initializeOptimization();
    odom_optimizer.optimize(30);
    odom_optimizer.save(pose_g2o);
    std::cout << "Saving the Odometry after optimizing!" << std::endl;
    UpdateMap();

    return 0;
}
