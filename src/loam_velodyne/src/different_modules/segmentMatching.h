//
// Created by lx on 18-3-15.
//

#ifndef PROJECT_SEGMENTMATCHING_H
#define PROJECT_SEGMENTMATCHING_H

#include "subMap.h"
#include <pcl/registration/icp.h>


const float min_time_diff = 1.0;
const float max_search_radius = 20.0;
const float max_dist_between_poles = 0.4;

bool id_set = false;
int id_last;
std::vector<cv::Point2f> centroid_frame;
pcl::PointCloud<pcl::PointXYZ> point_cloud_icp;
ros::Publisher background_map_pub;
float pose_estimate[3];
float pose_estimate_last[3];

Eigen::Vector3f measure_pose;
Eigen::Vector3f predict_pose;

int no_ekf;

Eigen::Matrix3f Af;

Eigen::Matrix3f Q;

Eigen::Matrix3f R;   //一次卡尔曼测量协方差矩阵

Eigen::Matrix3f pred_true_Covariance;   //一次卡尔曼预测值与真实值之间的协方差矩阵

Eigen::Matrix3f esti_true_Covariance;   // 1次卡尔曼测量值与真实值之前的协方差矩阵

Eigen::Matrix3f k_filter;   //一次卡尔曼系数矩阵

struct matchpair{
    int id1;
    int id2;
    float center1[3];
    float center2[3];
    float score;
};

struct segment{
    int size;
    int frame_id;
    float centroid[3] = {0};
    double time_stamp;
    double eigen_value_feature[7] = {0};
    pcl::PointCloud<pcl::PointXYZI> segcloud;
};

struct distance_lx
{
    float dis;
    cv::Point2f a;
    cv::Point2f b;
};

struct mat_mem{
    int x;
    int y;
    int num;
};

struct match
{
    cv::Point2f A;
    cv::Point2f B;
    cv::Point2f a;
    cv::Point2f b;
    float dist;
    int count;
};

class segmatch{
public:
    ros::Publisher source_seg_map_pub;

    int findNextId();

    void filterNearestSegment(std::vector<segment>& segment_cloud);

    void findMatchesByFeature(segment& seg1, segment& seg2,
                              std::vector<matchpair>& matched_segments);

    void addSegmentToSource(std::vector<segment>& seg_to_add);

    float calculateCentroidDis(segment& seg1, segment& seg2);

    void updatePose();

    void linkMatchesCentroid(std::vector<matchpair>& pairs);

    void findMatchesInFrames(std::vector<segment> segments_in_frame);

    void matchCorrespondingFrames(std::vector<cv::Point2f> map1,
                                  std::vector<cv::Point2f> map2,
                                  double time_stamp);

    void loadMap();

    Eigen::Matrix4f matchSegmentsIcp(pcl::PointCloud<pcl::PointXYZ> cloud_icp, float x, float y, float yaw);

private:
    const float min_dis_between_centroid = 2.0f;
    const float max_dis_between_features = 1.0f;
    const int min_num_links = 5;
    const int min_match_num = 6;
    int frame_id_last_;
    bool frame_id_set = false;
    std::vector<segment> segment_cloud_;
    pcl::PointCloud<pcl::PointXYZI> source_cloud_map;
    std::unordered_map<int, std::vector<segment>> source_segment_cloud_;
    std::unordered_map<int, segment> source_segment_cloud_pcd_;
//    pcl::PointCloud<Point> segment_point_cloud_pcd_1;
//    pcl::PointCloud<Point> segment_point_cloud_pcd_2;
//    pcl::PointCloud<Point> segment_point_cloud_pcd_3;
//    pcl::PointCloud<Point> segment_point_cloud_pcd_4;

};
#endif //PROJECT_SEGMENTMATCHING_H
