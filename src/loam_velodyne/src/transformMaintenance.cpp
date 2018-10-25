#include <cmath>
#include "tinyxml.h"

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include "segmenter.h"
#include "Odometry.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

//typedef pcl::PointXYZ Point;
//typedef pcl::PointXYZI PointI;
//typedef pcl::PointCloud<Point> PointCloud;
//typedef pcl::PointCloud<PointI> PointICloud;
//typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
//typedef pcl::PointCloud<PointI>::Ptr PointICloudPtr;

using namespace std;
    typedef __SIZE_TYPE__ size_t;

    static float segment_id = 0;
    static bool aft_mapped_pose_updated = false;

    static int frame_id = 1;
    float score = 0.1;

    float transformSum[6] = {0};
    float transformIncre[6] = {0};
    float transformMapped[6] = {0};
    float transformMappedLast[6] = {0};
    float transformBefMapped[6] = {0};
    float transformAftMapped[6] = {0};

    ///New version of the modified part, including different classes;
    double cur_time = 0, cur_vel = 0, cur_theta = 0;
    double last_time = 0, last_vel = 0, last_theta = 0;
    const float dist_to_build_local_map = 8;
    nav_msgs::Odometry obd_msg;
    sensor_msgs::Imu Imu_msg;

    static bool IsSetInitalImu = false;
    static double init_theta = 0;
    ///OdomPart
    static Eigen::Matrix4f pose_ekf_mat;
    static Eigen::Vector3f pose_ekf_;
    static Eigen::Vector3f predict_pose_;
    static Eigen::Vector3f measure_pose_;
    //        nav_msgs::Path odometry_path_;

    static float last_vel_;
//    static geometry_msgs::Quaternion last_q_;
    static Eigen::Vector3f inc_distance_car;
    static double inc_distance;
    static double diff_time;
    static double diff_theta;

    static Eigen::Matrix3f Af;
    static Eigen::Matrix3f Q;
    static Eigen::Matrix3f R;                      //一次卡尔曼测量协方差矩阵
    static Eigen::Matrix3f pred_true_Covariance;   //一次卡尔曼预测值与真实值之间的协方差矩阵
    static Eigen::Matrix3f esti_true_Covariance;   // 1次卡尔曼测量值与真实值之前的协方差矩阵
    static Eigen::Matrix3f k_filter;               //一次卡尔曼系数矩阵

    ///odom part end;
    ///EKF Matrix params

    ///Feature Extract Part
    cv::Mat grid, grid_2, grid_3, temp_1, temp_2;
    double gWidth, gHeight;
    double miniGrid;
    int gridW, gridH;
    float clip_min_height, clip_max_height;
    float clip_min_height_2, clip_max_height_2;

    int dilation_size;
    std::vector<float> feature;     // 30=1+6+3+20
    std::vector<float> Pointnum;    // 1
    std::vector<float> Cov_mat;     // 6
    std::vector<float> Local_pose;  // 3
    std::vector<float> Slice_mat;   // 10
    float cov_scalar;
    int countPoints;
    float countPoints_top, countPoints_bottom, scalarPoints;

    struct poleslabel
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cv::Point3f center;
        int label;
    };

    ///Feature Extract Part

//    LxSlam::Odometry odom;
//    graphOptmizer graph;
    double time_stamp;

    ros::Publisher *pubLaserOdometry2Pointer = NULL;
    ros::Publisher *seg_points_pub = NULL;
    ros::Publisher *segment_pub = NULL;
    tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
    tf::TransformBroadcaster *tf_seg = NULL;
    nav_msgs::Odometry laserOdometry2;
    tf::StampedTransform laserOdometryTrans2;
    tf::StampedTransform Odometry_seg;

    std::string read_config_path;
    std::string local_map_save_path;
    std::ofstream posefile;
    string save_local_path;
    std::string save_pose;
    std::vector<sensor_msgs::PointCloud2> cloud_to_match;
    static int local_match_pointer = 0;
    static pcl::PointCloud<pcl::PointXYZI> cloud_for_cluster;

//    void read_xml() {
//
//        TiXmlDocument doc;
//        if (!doc.LoadFile(read_config_path)) {
//            std::cout << "error_xmltrans" << std::endl;
//        } else {
//            std::cout << "read_xml" << std::endl;
//        }
//
//        TiXmlElement *node = doc.FirstChildElement("lx_mapping");
//
//        TiXmlElement *local_map_save_path_Elem = node->FirstChildElement("local_map_save_path");
//
//        local_map_save_path = local_map_save_path_Elem->GetText();
//
////    save_local_path = local_map_save_path;
//        save_pose = local_map_save_path;
//        save_pose.append("mapping_pose.txt");
//        posefile.open(save_pose);
//
////    std::cout << "xml has been load!" << std::endl;
//    }

    bool swap_if_gt(float &a, float &b) {
        if (a > b) {
            std::swap(a, b);
            return true;
        }
        return false;
    }

    void MergeSensorData(sensor_msgs::Imu imu_data,
                    nav_msgs::Odometry vel_data,
                    double diff_time){
    //        geometry_msgs::Pose last_pose = odometry_path_.poses.end()->pose;

//        geometry_msgs::Quaternion cur_q = imu_data.orientation;

        double cur_vel_obd = vel_data.twist.twist.linear.x / 3.6;//km/h 转 m/s

        inc_distance = (cur_vel_obd + last_vel_) / 2 * diff_time;

//        diff_theta = -tf::getYaw(cur_q) + tf::getYaw(last_q_);

        diff_theta = cur_theta - last_theta;

        inc_distance_car << inc_distance * cos(diff_theta / 2.),
                inc_distance * sin(diff_theta / 2.), diff_theta;

        Eigen::Matrix3f car_rotate;
        car_rotate<< cosf(pose_ekf_[2]), -sinf(pose_ekf_[2]), 0,
                sinf(pose_ekf_[2]),  cosf(pose_ekf_[2]), 0,
                0, 0, 1;

        Eigen::Vector3f inc_distance_global;
        inc_distance_global = car_rotate * inc_distance_car;

        predict_pose_ = pose_ekf_ + inc_distance_global;

        std::cout << "inc_car :  "<< inc_distance_car << std::endl;
//        std::cout << "predict pose :  "<< predict_pose_ << std::endl;
        std::cout << "mea pose : " << measure_pose_ << std::endl;
        std::cout << "ekf pose :  "<< pose_ekf_ << std::endl;

//        last_q_ = cur_q;
        if(!IsSetInitalImu){
            last_theta = 0;
            IsSetInitalImu = true;
        }else{
            last_theta = cur_theta;
        }
        last_vel_ = cur_vel_obd;

    }

Eigen::Matrix4f tf_to_global_ekf(){

//    std::cout << "Begin output the EKF pose !" << std::endl;

    Af << 1, 0, -inc_distance * sinf(predict_pose_[2] + diff_theta/2.0),
            0, 1,  inc_distance * cosf(predict_pose_[2] + diff_theta/2.0),
            0, 0, 1;

    Q << 2 * diff_time, 0, 0,
            0, 2 * diff_time, 0,
            0, 0, 2 * diff_time;

    R << 5, 0, 0,
            0, 5, 0,
            0, 0, 5;

    pred_true_Covariance = Af * esti_true_Covariance * Af.transpose() + Q;

    k_filter = pred_true_Covariance * (pred_true_Covariance + R).inverse();
    Eigen::Vector3f diff_pose;
    diff_pose = measure_pose_ - predict_pose_;
    diff_pose[2] = atan2f(sinf(diff_pose[2]), cosf(diff_pose[2]));
    pose_ekf_ = predict_pose_ + k_filter * diff_pose;
    float mea_yaw = measure_pose_[2];
    float pre_yaw = predict_pose_[2];
    float k_yaw = k_filter(2, 2);

    pose_ekf_[2] = atan2f(k_yaw * sinf(mea_yaw) + (1 - k_yaw) * sinf(pre_yaw),
                          k_yaw * cosf(mea_yaw) + (1 - k_yaw) * cosf(pre_yaw));
    esti_true_Covariance = (Eigen::Matrix3f::Identity() - k_filter) * pred_true_Covariance;

    return pose_to_matrix(pose_ekf_[0], pose_ekf_[1], 0,
                          0, 0, pose_ekf_[2]);
}

void setMea(Eigen::Vector3f mea_pose) {
    measure_pose_ = mea_pose;
}

Eigen::Matrix4f get_ekf_pose() {
    pose_ekf_mat = tf_to_global_ekf();
    return pose_ekf_mat;
}


bool check_translation() {

    float diff_x = transformMapped[3] - transformMappedLast[3];
    float diff_y = transformMapped[4] - transformMappedLast[4];
//    float diff_z = transformMapped[5] - transformMappedLast[5];

    float dist = sqrtf(diff_x * diff_x + diff_y * diff_y);
    if (dist >= 0.1)
        return true;
    else
        return false;

}

/********************************************Feature Extract Part**********************************************/

void icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg)
{
    if (_binImg.empty() || _binImg.type() != CV_8UC1)
    {
        return;
    }

    cv::Mat edge_binImg = cv::Mat::zeros(_binImg.rows + 2, _binImg.cols + 2, CV_8UC1);

    _binImg.copyTo(edge_binImg(cv::Rect(1, 1, _binImg.cols, _binImg.rows)));

    _lableImg.release();
    edge_binImg.convertTo(_lableImg, CV_32SC1);

    int label = 1;  // start by 2

    int rows = _binImg.rows - 1;
    int cols = _binImg.cols - 1;

    for (int i = 1; i < rows - 1; i++)
    {
        auto data = _lableImg.ptr<int>(i);
        for (int j = 1; j < cols - 1; j++)
        {
            if (data[j] == 1)
            {
                std::stack<std::pair<int, int> > neighborPixels;
                neighborPixels.push(std::pair<int, int>(i, j));  // pixel position: <i,j>

                ++label;  // begin with a new label
                while (!neighborPixels.empty())
                {
                    // get the top pixel on the stack and label it with the same label
                    std::pair<int, int> curPixel = neighborPixels.top();
                    int curX = curPixel.first;
                    int curY = curPixel.second;
                    _lableImg.at<int>(curX, curY) = label;

                    // pop the top pixel
                    neighborPixels.pop();

                    //                         push the 4-neighbors (foreground pixels)
                    if (_lableImg.at<int>(curX, curY - 1) == 1)
                    {  // left pixel
                        neighborPixels.push(std::pair<int, int>(curX, curY - 1));
                    }
                    if (_lableImg.at<int>(curX, curY + 1) == 1)
                    {  // right pixel
                        neighborPixels.push(std::pair<int, int>(curX, curY + 1));
                    }
                    if (_lableImg.at<int>(curX - 1, curY) == 1)
                    {  // up pixel
                        neighborPixels.push(std::pair<int, int>(curX - 1, curY));
                    }
                    if (_lableImg.at<int>(curX + 1, curY) == 1)
                    {  // down pixel
                        neighborPixels.push(std::pair<int, int>(curX + 1, curY));
                    }
                }
            }
        }
    }

    cv::Mat tmp = _lableImg.clone();
    _lableImg.release();
    _lableImg = cv::Mat::zeros(tmp.rows - 2, tmp.cols - 2, CV_32SC1);
    tmp(cv::Rect(1, 1, _lableImg.cols, _lableImg.rows)).copyTo(_lableImg);
    tmp.release();
}

cv::Point2i trans(cv::Point2f pt)
{
    return cv::Point2i(int(pt.x / 0.15) + 100 / 2, int(pt.y / 0.15) + 100 / 2);
}

void clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
               pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr, float in_min_height, float in_max_height)
{
    pcl::PassThrough<pcl::PointXYZI> pass;               //设置滤波器对象
    pass.setInputCloud(in_cloud_ptr);                    //设置输入点云
    pass.setFilterFieldName("z");                        //设置过滤时所需要点云类型的z字段
    pass.setFilterLimits(in_min_height, in_max_height);  //设置在过滤字段上的范围
    pass.filter(*out_cloud_ptr);
}

void genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, cv::Mat& mat)
{
    mat.setTo(0);
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
        cv::Point2i tmpIdx = trans(cv::Point2f(tmpI.x, tmpI.y));

        if (tmpIdx.x >= 0 && tmpIdx.x < gridW && tmpIdx.y >= 0 && tmpIdx.y < gridH)
        {
            mat.at<uchar>(tmpIdx.y, tmpIdx.x) = 1;
        }
    }
}

void extractTree(const cv::Mat label, cv::Mat& mat)
{
    double min, max;
    minMaxIdx(label, &min, &max);
    std::vector<int> labelnum;
    labelnum.resize(max + 1, 0);
    mat.setTo(0);
    for (int i = 0; i < label.rows; ++i)
    {
        for (int j = 0; j < label.cols; ++j)
        {
            labelnum[label.at<int>(i, j)]++;
        }
    }

    for (int i = 0; i < label.rows; ++i)
    {
        for (int j = 0; j < label.cols; ++j)
        {
            if (labelnum[label.at<int>(i, j)] < 30)
                mat.at<uchar>(i, j) = 1;
        }
    }
}

void matormat(const cv::Mat temp1, const cv::Mat temp2, cv::Mat& temp3)
{
    temp3.setTo(0);
    for (int i = 0; i < temp1.rows; ++i)
    {
        for (int j = 0; j < temp1.cols; ++j)
        {
            if (temp1.at<uchar>(i, j) == 1 || temp2.at<uchar>(i, j) == 1)
                temp3.at<uchar>(i, j) = 1;
        }
    }
}

void features(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)
{
    //点数
    Pointnum.resize(1, 0);
    Pointnum[0] = cloud_->points.size();
    //协方差
    Cov_mat.resize(6, 0);
    float x_avr = 0;
    float y_avr = 0;
    float z_avr = 0;

    for (int pp = 0; pp < cloud_->points.size(); ++pp)
    {
        x_avr += cloud_->points[pp].x;
        y_avr += cloud_->points[pp].y;
        z_avr += cloud_->points[pp].z;
    }
    x_avr /= cloud_->points.size();
    y_avr /= cloud_->points.size();
    z_avr /= cloud_->points.size();

    for (int pp = 0; pp < cloud_->points.size(); ++pp)
    {
        Cov_mat[0] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].x - x_avr);  // cov(x,x)
        Cov_mat[1] += (cloud_->points[pp].y - y_avr) * (cloud_->points[pp].y - y_avr);  // cov(y,y)
        //            Cov_mat[2] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].y - y_avr);//cov(x,y)
        //            Cov_mat[3] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].z - z_avr);//cov(x,z)
        //            Cov_mat[4] += (cloud_->points[pp].y - y_avr) * (cloud_->points[pp].z - z_avr);//cov(y,z)
        //            Cov_mat[5] += (cloud_->points[pp].z - z_avr) * (cloud_->points[pp].z - z_avr);//cov(z,z)
    }

    for (int i = 0; i < 6; ++i)
    {
        Cov_mat[i] /= (cloud_->points.size() - 1);
    }

    float cov_min = 999, cov_max = -999;
    for (int i = 0; i < 2; ++i)
    {
        if (Cov_mat[i] < cov_min)
            cov_min = Cov_mat[i];
        if (Cov_mat[i] > cov_max)
            cov_max = Cov_mat[i];
    }
    cov_scalar = cov_max / cov_min;
    //切片
    float min_z = 100;
    float max_z = -100;
    for (int i = 0; i < cloud_->points.size(); i++)
    {
        if (cloud_->points[i].z < min_z)
            min_z = cloud_->points[i].z;
        if (cloud_->points[i].z > max_z)
            max_z = cloud_->points[i].z;
    }

    int sliceNum = 7;
    Slice_mat.resize(sliceNum * 2, 0);
    float sliceStep = (max_z - min_z) / sliceNum;

    countPoints = 0;
    countPoints_top = 0;
    countPoints_bottom = 0;
    scalarPoints = 0;
    if (sliceStep > 0.1)
    {
        std::vector<std::vector<pcl::PointXYZI> > sliceHistgram;
        sliceHistgram.resize(sliceNum);
        for (int i = 0; i < cloud_->points.size(); i++)
        {
            int sliceIndex = (int)((cloud_->points[i].z - min_z) / sliceStep);
            if (sliceIndex == sliceNum)
                sliceIndex -= 1;
            sliceHistgram[sliceIndex].push_back(cloud_->points[i]);
        }

        for (int i = 0; i < sliceHistgram.size(); i++)
        {
            if (sliceHistgram[i].size() == 0)
            {
                countPoints++;
            }

            if (i < 3)
                countPoints_bottom += sliceHistgram[i].size();
            else
                countPoints_top += sliceHistgram[i].size();
        }
        scalarPoints = countPoints_top / countPoints_bottom;
    }

    feature.clear();
    feature.push_back(Pointnum[0]);
    feature.push_back((max_z - min_z));
    feature.push_back(countPoints);
}

void genClusters2(const cv::Mat label, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                              std::vector<poleslabel>& poles)
{
    double min, max;
    minMaxIdx(label, &min, &max);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > tmpPCDs;
    tmpPCDs.resize(max);

    std::vector<int> labelnum;
    labelnum.resize(max + 1, 0);
    for (int i = 0; i < label.rows; ++i)
    {
        for (int j = 0; j < label.cols; ++j)
        {
            labelnum[label.at<int>(i, j)]++;
        }
    }

    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
        cv::Point2i tmpIndex = trans(cv::Point2f(tmpI.x, tmpI.y));
        if (tmpIndex.x >= 0 && tmpIndex.x < gridW && tmpIndex.y >= 0 && tmpIndex.y < gridH)
        {
            int index = label.at<int>(tmpIndex.y, tmpIndex.x);
            if (index <= 0)
                continue;
            if (labelnum[index] > 50)
                continue;
            tmpPCDs[index - 1].push_back(tmpI);
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr poles_posi(new pcl::PointCloud<pcl::PointXYZI>);
    poles_posi->clear();
    sensor_msgs::PointCloud2 output_poles;
    for (int i = 0; i < tmpPCDs.size(); ++i)
    {
        if (tmpPCDs[i].points.size() < 10)
            continue;
        float z_min = 999, z_max = -999;
        float x_pos = 0, y_pos = 0, z_pos = 0;
        std::vector<cv::Point2f> points;
        points.clear();
        for (int j = 0; j < tmpPCDs[i].points.size(); ++j)
        {
            if (tmpPCDs[i].points[j].z < z_min)
                z_min = tmpPCDs[i].points[j].z;
            if (tmpPCDs[i].points[j].z > z_max)
                z_max = tmpPCDs[i].points[j].z;
            x_pos += tmpPCDs[i].points[j].x;
            y_pos += tmpPCDs[i].points[j].y;
            z_pos += tmpPCDs[i].points[j].z;
            points.push_back(cv::Point2f(tmpPCDs[i].points[j].x, tmpPCDs[i].points[j].y));
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
        *cloud_ = tmpPCDs[i];
        x_pos /= tmpPCDs[i].points.size();
        y_pos /= tmpPCDs[i].points.size();
        z_pos /= tmpPCDs[i].points.size();

        //********利用判定来识别柱子
        if ((z_max - z_min) < 1.5)
            continue;
        features(cloud_);
        if (countPoints > 1)
            continue;
        //        std::cout<<"\033[32m the cov_scalar is \033[0m"<<cov_scalar<<"the scalar of points
        //        is:"<<scalarPoints<<std::endl;
        if (scalarPoints > 3)
            continue;
        if (cov_scalar > 3)
            continue;
        cv::RotatedRect rect = cv::minAreaRect(points);
        float long_size = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
        float short_size = rect.size.height > rect.size.width ? rect.size.width : rect.size.height;
        //        std::cout<<"the long/short is "<<long_size/short_size<<std::endl;
        if (long_size > 0.4 && long_size / short_size > 2)
            continue;
        //        std::cout << " the x and y position of poles is " << x_pos << "and " << y_pos << std::endl;
        poleslabel pole;
        pole.cloud = tmpPCDs[i];
        pole.center = cv::Point3f(x_pos, y_pos, z_max - z_min);
        pole.label = 1;
        poles.push_back(pole);
    }
}

bool ExtractPoles(pcl::PointCloud<pcl::PointXYZI> cloud,
                  float cluster_centroids[3],
                  pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position){
//    #if DEBUG_POLES_EXT
//    displayer2->publishPoints(pub_points, cloud, ros::Time::now());  // msg.header.stamp);
//    #endif
    float point_begin_x = 0;
    float point_begin_y = 0;
    for (int i = 0; i < 16; ++i)
    {
        if (powf(cloud.points.at(i * cloud.width).x, 2) + powf(cloud.points.at(i * cloud.width).y, 2) > 0.1)
        {
            point_begin_x = cloud.points.at(i * cloud.width).x;
            point_begin_y = cloud.points.at(i * cloud.width).y;
            break;
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr_2(new pcl::PointCloud<pcl::PointXYZI>);

    //利用栅格法进行两个高度分割
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
    clipCloud(cloud_ptr, clipped_cloud_ptr, clip_min_height, clip_max_height);
    clipCloud(cloud_ptr, clipped_cloud_ptr_2, clip_min_height_2, clip_max_height_2);

    genGrid(clipped_cloud_ptr, grid);
    genGrid(clipped_cloud_ptr_2, grid_2);

    dilation_size = 1;
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                            cv::Point(dilation_size, dilation_size));
    dilate(grid, grid, element);
    dilate(grid_2, grid_2, element);

    cv::Mat label;  // int型
    icvprCcaBySeedFill(grid, label);

    cv::Mat label_2;
    icvprCcaBySeedFill(grid_2, label_2);

    //将柱状分割提取出来
    extractTree(label, temp_1);
    extractTree(label_2, temp_2);

    //将两个高度融合一起
    matormat(temp_1, temp_2, grid_3);

    cv::Mat label_3;
    icvprCcaBySeedFill(grid_3, label_3);
    //进一步判断，将柱状物体分割出来
    std::vector<poleslabel> poles;

    genClusters2(label_3, cloud_ptr, poles);

    for (int i = 0; i < poles.size(); ++i)
    {
        pcl::PointXYZI temp_poles;
        temp_poles.x = poles[i].center.x;
        temp_poles.y = poles[i].center.y;
        temp_poles.z = poles[i].center.z;
        poles_position->points.push_back(temp_poles);
    }

}
/***********************************Extract Part*****************************************************/

//    void SaveToPCD(pcl::PointCloud<pcl::PointXYZI> cloud, int id) {
//
//        stringstream ss;
//        save_local_path = local_map_save_path;
//        ss << id;
//        string frame_num = ss.str();
//        frame_num.append("laser.pcd");
//        save_local_path.append(frame_num);
//        std::vector<int> indicies;
//        pcl::removeNaNFromPointCloud(cloud, cloud, indicies);
//
//        pcl::io::savePCDFileASCII(save_local_path, cloud);
//        std::cout << "local_laser has been saved!!" << std::endl;
//    }

    void transformAssociateToMap() {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
                   - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
                   + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz)
                    - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly)
                                     - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly)
                    - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz)
                                     - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly)
                               - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx)
                       - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz)
                                        + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) -
                                        calx * cblx * cbly * saly)
                       + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz)
                                        + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) +
                                        calx * cblx * saly * sbly);
        float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz)
                               - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx)
                       + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx)
                                        + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) +
                                        calx * caly * cblx * cbly)
                       - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly)
                                        + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) -
                                        calx * caly * cblx * sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]),
                                   crycrx / cos(transformMapped[0]));

        float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz)
                                                             - calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                                                             cblx * cbly * salx)
                       - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly)
                                                               - calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                                                               cblx * salx * sbly)
                       + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly)
                                                             - calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                                                             cblx * salx * sbly)
                       - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz)
                                                               - calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                                                               cblx * cbly * salx)
                       + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]),
                                   crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3]
                             - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5]
                             - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        cur_time = laserOdometry->header.stamp.toSec();

        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;

        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;

        transformAssociateToMap();

/*************************************************************************************************************************/

        auto cloud_iter = cloud_to_match.begin();
        for (; cloud_iter < cloud_to_match.begin() + 10; ++cloud_iter) {

            ///Choose the corresponding pointcloud of the same timestamp
            double check_time = fabs(cloud_iter->header.stamp.toSec() - laserOdometry->header.stamp.toSec());
            if (check_time == 0 && !cloud_iter->data.empty() && check_translation()) {

                ///Begin the odom EKF part to get the ekf pose here;
                double diff_time = cur_time - last_time;
//                std::cout << "transform pose :  "<< transformMapped[0] << " " << transformMapped[1] << " "
//                        << transformMapped[2] << " " << transformMapped[3] << " " << transformMapped[4] << " "
//                        << transformMapped[5] << " " << std::endl;
//                Eigen::Vector3f mea_pose = {transformMapped[3], transformMapped[4], -transformMapped[1]};
//
//                setMea(mea_pose);
////                std::cout <<"Begin the EKF part"<< std::endl;
//
//                MergeSensorData(Imu_msg, obd_msg, diff_time);
//                Eigen::Matrix4f ekf_pose_mat = get_ekf_pose();
//                Eigen::Vector3d ekf_pose;

                pcl::PointCloud<pcl::PointXYZI> cloud_to_save;
                pcl::fromROSMsg(*cloud_iter, cloud_to_save);

                ///Eigen::Matrix4f transform = pose_to_matrix(ekf_pose[0], ekf_pose[1], 0, 0, 0, ekf_pose[2]);
//                double roll_,pitch_,z,x,y,yaw_;
//                matrix_to_pose(ekf_pose_mat, x, y, z, roll_, pitch_, yaw_);
//
//                transformMapped[3] = (float)x;
//                transformMapped[4] = (float)y;
//                transformMapped[1] = -(float)yaw_;

                std::cout << "Output the ekf pose x:"<<transformMapped[3]
                          << "  y: " << transformMapped[4] << std::endl;

//                if(frame_id == 1){
//                    graph.init();
//                }else{
//                    Id id1,id2;
//                    id1.counter = frame_id - 1;
//                    id2.counter = frame_id;
//                    id1.time_ns = id2.time_ns = time_stamp;
//                    graph.addVertexToGraph(ekf_pose, time_stamp);
//                    graph.addEdgeToGraph(ekf_pose_mat, id1, id2);
//                }
                ///对应方式不是上面的getQuat函数里面的对应关系

                Eigen::Matrix4f transform = pose_to_matrix(transformMapped[3],transformMapped[4],transformMapped[5],
                                                           transformMapped[1],-transformMapped[0],transformMapped[2]);
                ///记住上面的对应关系！！！！！！！很重要！！！！！！！

                pcl::PointCloud<pcl::PointXYZI> cloud_to_add;
                pcl::transformPointCloud(cloud_to_save, cloud_to_add, transform);///这里是先累加再进行滤波操作

                float dist_between_last_trans = std::hypotf(fabs(transformMapped[3] - transformMappedLast[3]),
                                                            fabs(transformMapped[4] - transformMappedLast[4]));
                if (dist_between_last_trans < dist_to_build_local_map ) {
                    std::cout<< "add fucking map "<< dist_between_last_trans << std::endl;
                    cloud_for_cluster += cloud_to_add;
                    break;
                }///Add several frames together to make the clusters' pointcloud dense.
                else {
                    ///SaveToPCD(cloud_to_save , frame_id);

                    ///save the pose of the single frame
//                    *cloud_filtered_ptr = cloud_for_cluster;

                    /******************************segment************************************/

                    std::cout << "Start the voxel filter part" << std::endl;

                    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
                    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem_;
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_filter_ptr
                            (new pcl::PointCloud<pcl::PointXYZI>());

                    *cloud_to_filter_ptr = cloud_for_cluster;
//                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr
//                            (new pcl::PointCloud<pcl::PointXYZI>());

                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr
                            (new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_
                            (new pcl::PointCloud<pcl::PointXYZI>());

                    voxel_filter_.setInputCloud(cloud_to_filter_ptr);
                    voxel_filter_.setLeafSize(0.2, 0.2, 0.2);
                    voxel_filter_.filter(*cloud_filtered_ptr);
                    std::cout << "Start the dist filter part" << std::endl;
                    std::cout<< "size of filter cloud : " << cloud_filtered_ptr->points.size() << std::endl;

                    for (pcl::PointXYZI point : cloud_filtered_ptr->points) {
                        if ((point.z - transformMapped[5]) > -1.7 &&
                            (point.z - transformMapped[5]) < 4.0) {
                            if (std::hypotf((point.x - transformMapped[3]), (point.y - transformMapped[4]) < 30)) {
                                filtered_cloud_->push_back(point);
                            }
                        }
                    }
                    ///Radius and height filters to limit the amount of the processing points

//                    std::cout << "Dist filter cloud size : " << filtered_cloud_->size() << std::endl;
//
//                    std::cout << "the size of segcloud is : " << filtered_cloud_->size() << std::endl;
//                    filtered_cloud_->header.frame_id = "/camera_init";
//                    filtered_cloud_->header.stamp = cur_time;

//                    sensor_msgs::PointCloud2 seg_cloud_msg;
//                    pcl::toROSMsg(*filtered_cloud_,seg_cloud_msg);
//                    seg_points_pub->publish(seg_cloud_msg);

                    outrem_.setInputCloud(filtered_cloud_);
                    outrem_.setRadiusSearch(0.5);
                    outrem_.setMinNeighborsInRadius(30);
                    outrem_.filter(*filtered_cloud_);
//                    pcl::PointCloud<pcl::PointXYZI> segment_filter_cloud;
//                    segment_filter_cloud = segmenter.filterInputCloud();


                    std::cout << "Seg the cloud now" << std::endl;

                    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extractor_;

                    std::vector<pcl::PointIndices> cluster_indices;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_seg_ptr
                            (new pcl::PointCloud<pcl::PointXYZ>());

                    pcl::copyPointCloud(*filtered_cloud_, *filtered_cloud_seg_ptr);

                    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree
                            (new pcl::search::KdTree<pcl::PointXYZ>());
                    kd_tree->setInputCloud(filtered_cloud_seg_ptr);

                    euclidean_cluster_extractor_.setClusterTolerance(0.3);
                    euclidean_cluster_extractor_.setMinClusterSize(300);
                    euclidean_cluster_extractor_.setMaxClusterSize(12000);
                    euclidean_cluster_extractor_.setSearchMethod(kd_tree);

                    euclidean_cluster_extractor_.setInputCloud(filtered_cloud_seg_ptr);
                    euclidean_cluster_extractor_.extract(cluster_indices);

                    std::cout << "the size of indicies : " << cluster_indices.size() << std::endl;
                    int segment_id_ = 1;
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>());
                    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
                      it != cluster_indices.end(); ++it) {
                        pcl::PointCloud<pcl::PointXYZI> segmented_cluster;
                        float x_all = 0;
                        float y_all = 0;
                        float z_all = 0;
                        int size = 0;

                        for (auto pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                            filtered_cloud_->points[*pit].intensity = segment_id_;
                            cloud_cluster->points.push_back(filtered_cloud_->points[*pit]);
                            segmented_cluster.push_back(filtered_cloud_->points[*pit]);
                            x_all += filtered_cloud_->points[*pit].x;
                            y_all += filtered_cloud_->points[*pit].y;
                            z_all += filtered_cloud_->points[*pit].z;
                            size++;
                        }
                        float centroid[3];
                        centroid[0] = x_all / size;
                        centroid[1] = y_all / size;
                        centroid[2] = z_all / size;

                        pcl::PointCloud<pcl::PointXYZI>::Ptr poles_pose(new pcl::PointCloud<pcl::PointXYZI>());
                        ///Feature extraction part
//                        if(ExtractPoles(segmented_cluster,centroid,poles_pose)){
//
//                        }
                        ///Feature extraction part

                        segment_id_++;
                    }

                    std::cout << "the size of segcloud is : " << cloud_cluster->size() << std::endl;
                    cloud_cluster->header.frame_id = "/camera_init";
                    cloud_cluster->header.stamp = cur_time;

                    sensor_msgs::PointCloud2 seg_cloud_msg;
                    pcl::toROSMsg(*cloud_cluster,seg_cloud_msg);
                    seg_points_pub->publish(seg_cloud_msg);

                    /******************************segment************************************/

                    frame_id++;

                    transformMappedLast[0] = transformMapped[0];
                    transformMappedLast[1] = transformMapped[1];
                    transformMappedLast[2] = transformMapped[2];
                    transformMappedLast[3] = transformMapped[3];
                    transformMappedLast[4] = transformMapped[4];
                    transformMappedLast[5] = transformMapped[5];

                    last_time = cur_time;
                    last_vel = cur_vel;

                    cloud_for_cluster.clear();
                    aft_mapped_pose_updated = false;
                    break;
                }
            }
        }

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                (transformMapped[2], -transformMapped[0], -transformMapped[1]);
//    geoQuat = tf::createQuaternionMsgFromRollPitchYaw
//            (0, 0, 0);

        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry2.pose.pose.orientation.z = geoQuat.x;
        laserOdometry2.pose.pose.orientation.w = geoQuat.w;
        laserOdometry2.pose.pose.position.x = transformMapped[3];
        laserOdometry2.pose.pose.position.y = transformMapped[4];
        laserOdometry2.pose.pose.position.z = transformMapped[5];
        pubLaserOdometry2Pointer->publish(laserOdometry2);

        time_stamp = laserOdometry->header.stamp.sec;

        laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        tfBroadcaster2Pointer->sendTransform(laserOdometryTrans2);

        last_time = cur_time;

/***********************************************************************************************************************/
    }

    void obdCallback(const nav_msgs::OdometryConstPtr &vel_msg) {
        obd_msg = *vel_msg;
//        std::cout <<"Get the obd"<< std::endl;
    }

    void imuCallback(sensor_msgs::Imu imu_msg) {
        geometry_msgs::Quaternion q = imu_msg.orientation;
        cur_theta = -tf::getYaw(q);
        Imu_msg = imu_msg;
//        std::cout <<"Get the imu data here "<< std::endl;
        if(!IsSetInitalImu){
            init_theta = cur_theta;
            last_theta = cur_theta;
//        }else{
//            cur_theta = cur_theta - init_theta;
////            std::cout << "init theta : " << init_theta << std::endl;
        }
    }

    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped) {
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformAftMapped[0] = -pitch;
        transformAftMapped[1] = -yaw;
        transformAftMapped[2] = roll;

        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

        transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
        transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
        transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

        transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
        transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
        transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;

        aft_mapped_pose_updated = true;
    }

    void LocalLaserHandler(sensor_msgs::PointCloud2 cloud) {

        cloud_to_match.at(local_match_pointer) = cloud;

        local_match_pointer = (local_match_pointer + 1) % 5;

    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "transformMaintenance");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

//        private_nh.getParam("read_config_path", read_config_path);
        ///read_xml();

        cloud_to_match.resize(10);

        ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
                ("/laser_odom_to_init", 5, laserOdometryHandler);

        ros::Subscriber subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>
                ("/aft_mapped_to_init", 5, odomAftMappedHandler);

        ros::Subscriber subLocalLaser = nh.subscribe<sensor_msgs::PointCloud2>
                ("/rslidar_points", 2, LocalLaserHandler);

        ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry>
                ("/integrated_to_init", 5);

        /*****************************************************************************************/

        ros::Subscriber obd_sub = nh.subscribe<nav_msgs::Odometry>("/speed", 10, obdCallback);

        ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 10, imuCallback);

        ros::Publisher seg_points_pub2 = nh.advertise<sensor_msgs::PointCloud2>("/seg_cloud", 5);
        seg_points_pub = &seg_points_pub2;

//    ros::Publisher segment_pub2 = nh.advertise<loam_velodyne::SegmentCloud>("/segments_to_detect", 5);
//    segment_pub = &segment_pub2;

//        tf::TransformBroadcaster tf_seg2;
//        tf_seg = &tf_seg2;
//        Odometry_seg.frame_id_ = "/rs_odom";
//        Odometry_seg.child_frame_id_ = "/base_link2";
        /*****************************************************************************************/

        pubLaserOdometry2Pointer = &pubLaserOdometry2;
        laserOdometry2.header.frame_id = "/camera_init";
        laserOdometry2.child_frame_id = "/camera";

        tf::TransformBroadcaster tfBroadcaster2;
        tfBroadcaster2Pointer = &tfBroadcaster2;
        laserOdometryTrans2.frame_id_ = "/camera_init";
        laserOdometryTrans2.child_frame_id_ = "/camera";

        ros::spin();

        return 0;
    }
