//
// Created by timmy on 17-9-21.
//

#include "subMap.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

static int submapIndex = 0;
const int submapLength = 120;
static int numscans = 0;
const int max_iter_ndt = 30;

int count = 0;

static Eigen::Matrix4f initial_guess_imu = Eigen::Matrix4f::Identity();
//The transform matrix derived from the imu sensor
Eigen::Matrix4f ekf_tf_to_global = Eigen::Matrix4f::Identity();
//The transform matrix derived from the EKF algorithm
Eigen::Matrix4f ndt_tf_to_local = Eigen::Matrix4f::Identity();
Eigen::Matrix4f ndt_tf_to_global = Eigen::Matrix4f::Identity();
Eigen::Matrix4f imu_tf_to_local = Eigen::Matrix4f::Identity();
Eigen::Matrix4f imu_tf_to_global = Eigen::Matrix4f::Identity();

PointCloud submap_old;
//Submap used for insertion of the pointcloud data
PointCloud submap_new;
//Submap to store the data in the late data in submap_old
static bool submap_init = false;
//Flag to show if the first submap has been written or not

std::vector<PointCloud> SubMapsForLoop;
std::vector<Pose> PoseForLoop;
std::vector<PointCloud> PointCloudForLoop;
ros::Publisher submap_pub;

Pose imu_localizer = {0,0,0,0,0,0};
Pose ndt_localizer = {0,0,0,0,0,0};
Pose ekf_pose = {0,0,0,0,0,0};
Pose pose;
std::vector<Pose> poses;

const int imuQueLength = 50;
double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

int imuPointerLast = -1;
const float scanPeriod = 0.1;
static float diff_time = 0.05;

Eigen::MatrixXf cov_pre = Eigen::MatrixXf::Identity(6,6);
Eigen::MatrixXf cov_true = Eigen::MatrixXf::Identity(6,6);
const Eigen::MatrixXf cov_identity = Eigen::MatrixXf::Identity(6,6);

void featureCloudHandler
        (const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

    PointCloud laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    PointCloud::Ptr laserCloudInPtr
            (new pcl::PointCloud<pcl::PointXYZ>(laserCloudIn));
    pcl::VoxelGrid<pcl::PointXYZ> Voxelfilter;
    Voxelfilter.setInputCloud( laserCloudInPtr );
    Voxelfilter.setLeafSize(0.4f, 0.4f, 0.4f);
    Voxelfilter.filter(laserCloudIn);

    InsertToSubmap(laserCloudIn);

}

void AddSubmap(){

    SubMapsForLoop.push_back(PointCloudForLoop[submapIndex]);
    PoseForLoop.push_back(ekf_pose);
}   //添加子图存储进内存用于后续的调用回环检测

Eigen::Matrix4f ICPMatch(PointCloud lastPointCloud,
                         PointCloud curPointCloud){

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud::Ptr lastPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(lastPointCloud));
    PointCloud::Ptr curPointCloudPtr((new pcl::PointCloud<pcl::PointXYZ>(curPointCloud)));

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setMaximumIterations (300);

    icp.setInputSource (curPointCloudPtr);
    icp.setInputTarget (lastPointCloudPtr);
    icp.align (*output_cloud, initial_guess_imu);

    result = icp.getFinalTransformation();
    return  result;

}

void EKF(){

    Eigen::MatrixXf cov_R = Eigen::MatrixXf::Identity(6,6) * diff_time;
    Eigen::MatrixXf cov_Q = Eigen::MatrixXf::Identity(6,6);
    Eigen::MatrixXf Af = Eigen::MatrixXf::Identity(6,6);
    Eigen::MatrixXf KalmanGain = Eigen::MatrixXf::Identity(6,6);
    Eigen::VectorXf ndt_measurement(6,1);
    Eigen::VectorXf pose_pre(6,1), ekf_result(6,1);

    Af = cov_identity;
    //TODO: Make the transformation matrix differentiable
    cov_Q << 10., 0., 0., 0., 0., 0.,
            0., 10., 0., 0., 0., 0.,
            0., 0., 10., 0., 0., 0.,
            0., 0., 0., 30., 0., 0.,
            0., 0., 0., 0., 30., 0.,
            0., 0., 0., 0., 0., 30.;

    ndt_measurement[0] = ndt_localizer.x;
    ndt_measurement[1] = ndt_localizer.y;
    ndt_measurement[2] = ndt_localizer.z;
    ndt_measurement[3] = ndt_localizer.roll;
    ndt_measurement[4] = ndt_localizer.pitch;
    ndt_measurement[5] = ndt_localizer.yaw;

    pose_pre[0] = imu_localizer.x;
    pose_pre[1] = imu_localizer.y;
    pose_pre[2] = imu_localizer.z;
    pose_pre[3] = imu_localizer.roll;
    pose_pre[4] = imu_localizer.pitch;
    pose_pre[5] = imu_localizer.yaw;

    cov_pre = Af * cov_true * Af.transpose() + cov_R;
    KalmanGain = cov_pre * (cov_pre + cov_Q).inverse();
    cov_true = (cov_identity - KalmanGain) * cov_pre;
    ekf_result = pose_pre + KalmanGain * (ndt_measurement - pose_pre);
    //Kalman filter formula

    ekf_pose.x = ekf_result[0];
    ekf_pose.y = ekf_result[1];
    ekf_pose.z = ekf_result[2];
    ekf_pose.roll = ekf_result[3];
    ekf_pose.pitch = ekf_result[4];
    ekf_pose.yaw = ekf_result[5];

    Eigen::AngleAxisf init_rotation_x( ekf_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y( ekf_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z( ekf_pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(ekf_pose.x, ekf_pose.y, ekf_pose.z);
    ekf_tf_to_global =
            (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
//    ekf_tf_to_global =
//            (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix()
//            * ekf_tf_to_global;

    std::cout << "EKF is done" << std::endl;
    std::cout << "x " << ekf_pose.x << " y " << ekf_pose.y << " "
              << numscans << " " << count <<std::endl;

}

void InsertToSubmap(PointCloud pointcloud){

    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());

    if(submapIndex == 0 && numscans == 0) {
        ndt_tf_to_global = Eigen::Matrix4f::Identity();
    }
    else {
//        double start = std::clock();
        std::cout << "pointcloud size is : " << pointcloud.points.size() << std::endl;
        //ndt_tf_to_local = NdtMatch(submap_old, pointcloud);
        ndt_tf_to_local = ICPMatch(submap_old, pointcloud);
//        double end = std::clock();
//        std::cout << "NDT" << std::endl;
    }

    tf::Matrix3x3 mat_rotate;
    mat_rotate.setValue(static_cast<double>(ndt_tf_to_local(0, 0)),
                        static_cast<double>(ndt_tf_to_local(0, 1)),
                        static_cast<double>(ndt_tf_to_local(0, 2)),
                        static_cast<double>(ndt_tf_to_local(1, 0)),
                        static_cast<double>(ndt_tf_to_local(1, 1)),
                        static_cast<double>(ndt_tf_to_local(1, 2)),
                        static_cast<double>(ndt_tf_to_local(2, 0)),
                        static_cast<double>(ndt_tf_to_local(2, 1)),
                        static_cast<double>(ndt_tf_to_local(2, 2)));

    mat_rotate.getRPY(ndt_localizer.roll, ndt_localizer.pitch, ndt_localizer.yaw, 1);
    ndt_localizer.x = ekf_pose.x + ndt_tf_to_local(0,3);
    ndt_localizer.y = ekf_pose.y + ndt_tf_to_local(1,3);
    ndt_localizer.z = ekf_pose.z + ndt_tf_to_local(2,3);

    std::cout << "ndt traslation : x " << ndt_tf_to_local(0,3)
              << "\n y : " << ndt_tf_to_local(1,3)
              << "\n z : " << ndt_tf_to_local(2,3)
                         << std::endl;

    EKF();
    //Use EKF to get the estimate pose combining both imu and ndt data

    pcl::transformPointCloud(pointcloud, *output_cloud, ekf_tf_to_global);

    if(!submap_init){
        submap_old += *output_cloud;
        if(numscans == submapLength / 2 ){
            submap_init = true;
        }
    }
    else{
        if(numscans == submapLength - 1){
            submap_old = submap_new;
            submap_new.clear();
            submap_old += *output_cloud;
            submap_new = *output_cloud;
        }
        else{
            submap_old += *output_cloud;
            submap_new += *output_cloud;
        }
    }
    //Fill the old and new submap at the same time, and when the old submap
    //is full, make the new submap to be the old and clear the new submap
    numscans++;

    if(numscans >= (submapLength - 10)){
        if(numscans == (submapLength - 10)){
            PointCloudForLoop.push_back(*output_cloud);
        } else
            PointCloudForLoop[submapIndex] += *output_cloud;
        if(numscans >= submapLength){
            AddSubmap();
            submapIndex++;
            numscans = 0;
        }
    }
    count++;
    //Save the latest 25 scans in the submap data which will be
    //used for detect loop closure

}   //根据imu数据的估值将当前帧数据与之前的数据进行匹配，用来将当前帧插入子图

Eigen::Matrix4f NdtMatch(PointCloud lastPointCloud,
                         PointCloud curPointCloud){

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud::Ptr lastPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(lastPointCloud));
    PointCloud::Ptr curPointCloudPtr((new pcl::PointCloud<pcl::PointXYZ>(curPointCloud)));

    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setTransformationEpsilon(0.08);
    ndt.setStepSize(0.2);
    ndt.setResolution(0.7);
    ndt.setMaximumIterations(max_iter_ndt);
    ndt.setInputSource(curPointCloudPtr);
    ndt.setInputTarget(lastPointCloudPtr);
    std::cout << "what the hell?" << std::endl;
    ndt.align(*output_cloud, initial_guess_imu);
    std::cout << "what the hell2?" << std::endl;

    result = ndt.getFinalTransformation();
//    std::cout << "NDT ALIGN IS FINISHED!" << std::endl;
    return result;
}
//Use NDT to match the new scan and the current submap_old to get
//the transformation

void AccumulateIMUShift(){

    float roll = imuRoll[imuPointerLast];
    float pitch = imuPitch[imuPointerLast];
    float yaw = imuYaw[imuPointerLast];
    float accX = imuAccX[imuPointerLast];
    float accY = imuAccY[imuPointerLast];
    float accZ = imuAccZ[imuPointerLast];

    float x1 = cos(roll) * accX - sin(roll) * accY;
    float y1 = sin(roll) * accX + cos(roll) * accY;
    float z1 = accZ;

    float x2 = x1;
    float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    float z2 = sin(pitch) * y1 + cos(pitch) * z1;

    accX = cos(yaw) * x2 + sin(yaw) * z2;
    accY = y2;
    accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
    double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
    if (timeDiff < 1) {

        float imu_diff_x = imuVeloX[imuPointerBack] * timeDiff
                            + accX * timeDiff * timeDiff / 2;
        float imu_diff_y = imuVeloY[imuPointerBack] * timeDiff
                            + accY * timeDiff * timeDiff / 2;
        float imu_diff_z = imuVeloZ[imuPointerBack] * timeDiff
                            + accZ * timeDiff * timeDiff / 2;

        float imu_diff_roll = imuRoll[imuPointerLast] - imuRoll[imuPointerBack];
        float imu_diff_pitch = imuPitch[imuPointerLast] - imuPitch[imuPointerBack];
        float imu_diff_yaw = imuYaw[imuPointerLast] - imuYaw[imuPointerBack];

//        imu_diff_roll = 0;
//        imu_diff_pitch = 0;
//        if(imu_diff_yaw > M_PI / 16)
//            imu_diff_yaw = 0;

        imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imu_diff_x;
        imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imu_diff_y;
        imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imu_diff_z;

        imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
        imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
        imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
        //加速度积分得速度，速度积分得位移

        //std::cout << "AccumulateIMU DATA is finished! " << std::endl ;

        Eigen::AngleAxisf init_rotation_x(imu_diff_roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(imu_diff_pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(imu_diff_yaw, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f init_translation( imu_diff_x, imu_diff_y, imu_diff_z );

        initial_guess_imu =
                (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

//        std::cout << initial_guess_imu << std::endl;

        imu_localizer.roll = ekf_pose.roll + imuRoll[imuPointerLast] - imuRoll[imuPointerBack];
        imu_localizer.pitch = ekf_pose.pitch + imuPitch[imuPointerLast] - imuPitch[imuPointerBack];
        imu_localizer.yaw = ekf_pose.yaw + imuYaw[imuPointerLast] - imuYaw[imuPointerBack];

        imu_localizer.x = ekf_pose.x + imuShiftX[imuPointerLast];
        imu_localizer.y = ekf_pose.y + imuShiftY[imuPointerLast];
        imu_localizer.z = ekf_pose.z + imuShiftZ[imuPointerLast];
        //If the environment is assumed to be in the same horizon, the roll and the pitch can be
        //set to 0 to simplify the computation

        std::cout << "ndt traslation : x " << ndt_tf_to_local(0,3)
                  << "\n y : " << ndt_tf_to_local(1,3)
                  << "\n z : " << ndt_tf_to_local(2,3)
                  << std::endl;

//        std::cout << "Imu localization has been updated!" << std::endl;
//        std::cout << "numscans is : " << numscans << std::endl;
//        std::cout << "SubmapIndex is : " << submapIndex << std::endl;
        //Update IMU localizer.
    }
    //TODO: If the shift is out of threshold, we need to delete the data
    //Get initial guess of the pointcloud.
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){

    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
    float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
    float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;
    //消除各个轴上重力加速度产生的影响

    imuPointerLast = (imuPointerLast + 1) % imuQueLength;
//    std::cout << "imuPointerLast : " << imuPointerLast << std::endl;

    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast] = yaw;
    imuAccX[imuPointerLast] = accX;
    imuAccY[imuPointerLast] = accY;
    imuAccZ[imuPointerLast] = accZ;

    AccumulateIMUShift();
}

//Eigen::Matrix4f TransformToTransformation(float x, float y, float z,
//                                          float translation_x, float translation_y,
//                                          float translation_z){
//    Eigen::Matrix4f Transformation;
//
//    Transformation(0,0) = cos(y) * cos(z);
//    Transformation(0,1) = cos(z) * sin(x) * sin(y) - cos(x) * sin(z);
//    Transformation(0,2) = sin(z) * sin(x) + cos(x) * cos(z) * sin(y);
//    Transformation(1,0) = cos(y) * sin(z);
//    Transformation(1,1) = cos(z) * cos(x) + sin(x) * sin(z) * sin(y);
//    Transformation(1,2) = cos(x) * sin(y) * sin(z) - cos(z) * sin(x);
//    Transformation(2,0) = -sin(y);
//    Transformation(2,1) = cos(y) * sin(x);
//    Transformation(2,2) = cos(y) * cos(x);
//
//    Transformation(0,3) = translation_x;
//    Transformation(1,3) = translation_y;
//    Transformation(2,3) = translation_z;
//
//    Transformation(3,0) = 0;
//    Transformation(3,1) = 0;
//    Transformation(3,2) = 0;
//    Transformation(3,3) = 1;
//
//    return Transformation;
//}   //将读入的IMU数据转换为变换矩阵

void SubMap::callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
              const sensor_msgs::ImuConstPtr& imu_msg){

    double imu_time = imu_msg->header.stamp.toSec();
    double PointCloudTime = pointcloud_msg->header.stamp.toSec();
    double diffTime = fabs(PointCloudTime - imu_time);
    double all_start = std::clock();

    if(diffTime < scanPeriod){
        imuHandler(imu_msg);
        featureCloudHandler(pointcloud_msg);
    } else
        std::cout << "Timestamp cannot match!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

    double all_end = std::clock();
    std::cout << "All programme costs : " << all_end - all_start << std::endl;
    //For now the whole programme is about 25HZ 17.10.25
//    std::cout << CLOCKS_PER_SEC << std::endl;

//    sensor_msgs::PointCloud2 submapMsg;
//    pcl::toROSMsg(submap_old, submapMsg);
//    submapMsg.header.stamp = pointcloud_msg->header.stamp;
//    submapMsg.header.frame_id = "/camera";
//    submap_pub.publish(submapMsg);      //用于检验是否正常输出

}

SubMap::SubMap(ros::NodeHandle nh, ros::NodeHandle private_nh) {

    featurePoints_sub = new  message_filters::Subscriber
            <sensor_msgs::PointCloud2>(nh, "featurePoints", 2);

    imu_sub = new message_filters::Subscriber
            <sensor_msgs::Imu>(nh,"compensated_imu",50);

    sync_ = new  message_filters::Synchronizer<slamSyncPolicy>
            (slamSyncPolicy(10), *featurePoints_sub, *imu_sub);

    sync_->registerCallback(boost::bind(&SubMap::callback, this, _1, _2));
}


int main(int argc, char** argv){

    ros::init(argc, argv, "subMap");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    cov_true = cov_identity;

    submap_pub = nh.advertise<sensor_msgs::PointCloud2>
            ("/submap", 2);

    SubMap submap(nh,private_nh);

    ros::spin();

    return 0;
}

//std::cout << "what the hell?" << std::endl;