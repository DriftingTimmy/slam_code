//
// Created by lx on 17-12-8.
//

#include "tinyxml.h"

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <loam_velodyne/MatchedPair.h>
#include <loam_velodyne/KeyFrame.h>
#include "subMap.h"
#include "common_function.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

const int grid_width = 150;
const int grid_height = 150;
const float grid_cell_length = 0.2;
//const int density_range = 12;
const int sparsity = 3;
//const int match_threshold = 5;

static int frame_id = 0;
typedef cv::Mat Mat;
ros::Publisher markers_pub;
ros::Publisher matched_pub;

std::vector<sensor_msgs::PointCloud2> cloud_to_match_time;
const int local_match_length = 10;
static int local_match_pointer = 0;

//static double local_map_time;
static double global_map_time;
bool global_update_flag = false;

std::string read_config_path;
static int grid_map_feature_num;
static int grid_map_match_num;

void read_xml(){

    TiXmlDocument doc ;
    if(!doc.LoadFile(read_config_path))
    {
        std::cout<<"error_xml_BuildOccupancy"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = doc.FirstChildElement("lx_mapping") ;

    TiXmlElement* grid_map_feature_num_Elem       = node->FirstChildElement("grid_map_feature_num") ;
    TiXmlElement* grid_map_match_num_Elem         = node->FirstChildElement("grid_map_match_num") ;

    grid_map_feature_num    = atoi(grid_map_feature_num_Elem->GetText());
    grid_map_match_num      = atoi(grid_map_match_num_Elem->GetText());
}

struct grid_map{
    Mat mat;
    int frameId;
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

std::vector<std::vector<cv::Point2f>> grid_maps;

//void SaveToPCD(pcl::PointCloud<pcl::PointXYZI> cloud, int id){
//
//    string save_local_path = "/media/lx/文件及资料/Data/lx_map/loam_test/daxuecheng/";
//    stringstream ss;
//    ss << id;
//    string frame_num = ss.str();
//    frame_num.append("laser.pcd");
//    save_local_path.append(frame_num);
//
//    pcl::io::savePCDFileASCII (save_local_path, cloud);
//    std::cout << "local_laser has been saved!!" << std::endl;
//
//}

bool CompareNum(mat_mem tmp1, mat_mem tmp2){
    return tmp1.num > tmp2.num;
}

bool sort_distance_lx_dist(distance_lx a, distance_lx b)
{
    return a.dis < b.dis;
}

float dist_between(cv::Point2f a, cv::Point2f b)
{
    float dist = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
    return dist;
}

cv::Point2i TransToInd(cv::Point2f point){

    return cv::Point2i(int(point.x / grid_cell_length) + grid_width / 2,
                       int(point.y / grid_cell_length) + grid_height / 2);

}

bool sort_count(const match& a, const match& b)
{
    if (a.count != b.count)
        return a.count > b.count;
    else
        return a.dist < b.dist;
}

bool MatchCorrespondingGridMap(std::vector<cv::Point2f> map1,
                               std::vector<cv::Point2f> map2){

    std::vector<distance_lx> map1_distance_lx;
//    std::cout <<"bug test 111"<< std::endl;

    std::vector<cv::Point2f>::iterator it_ori = map1.begin();

//    std::cout << map1.size() << "  " << map2.size() << std::endl;

    for (; it_ori != map1.end() - 1; ++it_ori)
    {
        std::vector<cv::Point2f>::iterator it_ori2 = it_ori + 1;
        for (; it_ori2 != map1.end(); ++it_ori2)
        {
            float dis_ori = sqrt(powf((*it_ori).x - (*it_ori2).x, 2) +
                                         powf((*it_ori).y - (*it_ori2).y, 2));
            distance_lx temp_distance_lx;
            temp_distance_lx.a = (*it_ori);
            temp_distance_lx.b = (*it_ori2);
            temp_distance_lx.dis = dis_ori;
            map1_distance_lx.push_back(temp_distance_lx);
        }
    }

    std::vector<distance_lx> map2_distance_lx;
    std::vector<cv::Point2f>::iterator it_ori3 = map2.begin();
    for (; it_ori3 != map2.end() - 1; ++it_ori3)
    {
        std::vector<cv::Point2f>::iterator it_ori4 = it_ori3 + 1;

        for (; it_ori4 != map2.end(); ++it_ori4)
        {
            float dis_ori = sqrt(powf((*it_ori3).x - (*it_ori4).x, 2) +
                                         powf((*it_ori3).y - (*it_ori4).y, 2));
            distance_lx temp_distance_lx;
            temp_distance_lx.a = (*it_ori3);
            temp_distance_lx.b = (*it_ori4);
            temp_distance_lx.dis = dis_ori;
            map2_distance_lx.push_back(temp_distance_lx);
        }
    }

    std::stable_sort(map1_distance_lx.begin(), map1_distance_lx.end(), sort_distance_lx_dist);
    std::stable_sort(map2_distance_lx.begin(), map2_distance_lx.end(), sort_distance_lx_dist);

    std::vector<match> pairs_sample;

    std::vector<distance_lx>::iterator it_distance_lx1 = map1_distance_lx.begin();
    for (; it_distance_lx1 != map1_distance_lx.end(); ++it_distance_lx1)
    {
        float temp_distance_lx1 = (*it_distance_lx1).dis;

        float theta_it_distance_lx1 = atan2f((*it_distance_lx1).b.y - (*it_distance_lx1).a.y,
                                             (*it_distance_lx1).b.x - (*it_distance_lx1).a.x);
        std::vector<distance_lx>::iterator it_distance_lx2 = map2_distance_lx.begin();

        for (; it_distance_lx2 != map2_distance_lx.end(); ++it_distance_lx2){

            float temp_distance_lx2 = (*it_distance_lx2).dis;
            float theta_it_distance_lx2 = atan2f((*it_distance_lx2).b.y - (*it_distance_lx2).a.y,
                                                 (*it_distance_lx2).b.x - (*it_distance_lx2).a.x);
            float theta_it_distance_lx2_2 = atan2f((*it_distance_lx2).a.y - (*it_distance_lx2).b.y,
                                                   (*it_distance_lx2).a.x - (*it_distance_lx2).b.x);

            if(fabs(temp_distance_lx1 - temp_distance_lx2) < 0.2){
                match pairs_a_a, pairs_a_b;
                // a对a，b对b
                pairs_a_a.A = (*it_distance_lx1).a;
                pairs_a_a.B = (*it_distance_lx1).b;
                pairs_a_a.a = (*it_distance_lx2).a;
                pairs_a_a.b = (*it_distance_lx2).b;
                pairs_a_a.dist = fabs(temp_distance_lx1 - temp_distance_lx2);
                pairs_a_a.count = 0;

                float diff_trans_theta1 = atan2f(sinf(theta_it_distance_lx1 - theta_it_distance_lx2),
                                                       cosf(theta_it_distance_lx1 - theta_it_distance_lx2));

                float a_rotate_x = pairs_a_a.a.x * cosf(diff_trans_theta1) -
                                   pairs_a_a.a.y * sinf(diff_trans_theta1) ;
                float a_rotate_y = pairs_a_a.a.y * cosf(diff_trans_theta1) +
                                   pairs_a_a.a.x * sinf(diff_trans_theta1) ;

                float x_offset = pairs_a_a.A.x - a_rotate_x;
                float y_offset = pairs_a_a.A.y - a_rotate_y;

                std::vector<cv::Point2f>::iterator it_trans_to = map2.begin();
                for (; it_trans_to != map2.end(); ++it_trans_to)
                {
                    float tmp_trans_to_ori_x = (*it_trans_to).x * cosf(diff_trans_theta1) -
                                               (*it_trans_to).y * sinf(diff_trans_theta1) ;
                    float tmp_trans_to_ori_y = (*it_trans_to).y * cosf(diff_trans_theta1) +
                                               (*it_trans_to).x * sinf(diff_trans_theta1) ;

                    cv::Point2f it_trans_to_global;
                    it_trans_to_global.x = tmp_trans_to_ori_x + x_offset;
                    it_trans_to_global.y = tmp_trans_to_ori_y + y_offset;
                    std::vector<cv::Point2f>::iterator it_ori_1 = map1.begin();
                    for (; it_ori_1 != map1.end(); ++it_ori_1)
                    {
                        float dist = dist_between(it_trans_to_global, (*it_ori_1));
                        if (dist < 0.3)
                        {
                            pairs_a_a.count++;
                            break;
                        }
                    }
                }

                pairs_a_b.A = (*it_distance_lx1).a;
                pairs_a_b.B = (*it_distance_lx1).b;
                pairs_a_b.a = (*it_distance_lx2).b;
                pairs_a_b.b = (*it_distance_lx2).a;
                pairs_a_b.dist = fabs(temp_distance_lx1 - temp_distance_lx2);
                pairs_a_b.count = 0;

                float diff_trans_theta2 = atan2f(sinf(theta_it_distance_lx1 - theta_it_distance_lx2_2),
                                                 cosf(theta_it_distance_lx1 - theta_it_distance_lx2_2));

                float a_rotate_x_2 = pairs_a_b.a.x * cosf(diff_trans_theta2) -
                        pairs_a_b.a.y * sinf(diff_trans_theta2) ;
                float a_rotate_y_2 = pairs_a_b.a.y * cosf(diff_trans_theta2) +
                        pairs_a_b.a.x * sinf(diff_trans_theta2) ;

                float x_offset_2 = pairs_a_b.A.x - a_rotate_x_2;
                float y_offset_2 = pairs_a_b.A.y - a_rotate_y_2;

                std::vector<cv::Point2f>::iterator it_trans_to2 = map2.begin();
                for (; it_trans_to2 != map2.end(); ++it_trans_to2)
                {
                    float tmp_trans_to_ori_x = (*it_trans_to2).x * cosf(diff_trans_theta2) -
                                               (*it_trans_to2).y * sinf(diff_trans_theta2) ;
                    float tmp_trans_to_ori_y = (*it_trans_to2).y * cosf(diff_trans_theta2) +
                                               (*it_trans_to2).x * sinf(diff_trans_theta2) ;

                    cv::Point2f it_trans_to_global;
                    it_trans_to_global.x = tmp_trans_to_ori_x + x_offset_2;
                    it_trans_to_global.y = tmp_trans_to_ori_y + y_offset_2;
                    std::vector<cv::Point2f>::iterator it_ori_1 = map1.begin();
                    for (; it_ori_1 != map1.end(); ++it_ori_1)
                    {
                        float dist = dist_between(it_trans_to_global, (*it_ori_1));
                        if (dist < 0.3)
                        {
                            pairs_a_b.count++;
                            break;
                        }
                    }
                }

                pairs_sample.push_back(pairs_a_a);
                pairs_sample.push_back(pairs_a_b);
            }
        }
    }

//    std::cout << "bug test 4 " << std::endl;
    std::stable_sort(pairs_sample.begin(), pairs_sample.end(), sort_count);

    match the_one = (*pairs_sample.begin());
    float pairs_trans_theta = atan2f(the_one.b.y - the_one.a.y, the_one.b.x - the_one.a.x);
    float pairs_oir_theta = atan2f(the_one.B.y - the_one.A.y, the_one.B.x - the_one.A.x);
    // a-b逆时钟旋转角度到地图A-B,车的姿态也要加上这个旋转角度（逆时针）
    float pairs_diffs = atan2f(sinf(pairs_oir_theta - pairs_trans_theta),
                               cosf(pairs_oir_theta - pairs_trans_theta));
    //    std::cout << std::endl << "the trans theta is " << pairs_diffs / 3.1415 * 180 << std::endl;

//    float x0 = the_one.a.x * cosf(pairs_diffs) - the_one.a.y * sinf(pairs_diffs);
//    float y0 = the_one.a.y * cosf(pairs_diffs) + the_one.a.x * sinf(pairs_diffs);

    std::cout << "Grid match counter : " << the_one.count << std::endl;
//TODO: Try to provide the rotate and translation estimation of the mathced two grid maps.
    if(the_one.count >= grid_map_match_num){

        std::cout << "The grid map has been matched!!!!!!!" << std::endl;
        return true;
    }
    else{
        std::cout << "Failed to match the grid" << std::endl;
        return false;
    }
}

void PointCloudHandler(sensor_msgs::PointCloud2 pointcloud){

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(pointcloud, input_cloud);
    Mat grid_to_sort = cv::Mat::zeros(grid_height, grid_width, CV_8UC1);
    Mat grid_to_match = cv::Mat::zeros(grid_height, grid_width, CV_8UC1);
    Mat grid_cell_picked = cv::Mat::zeros(grid_height, grid_width, CV_8UC1);
    visualization_msgs::MarkerArray markers;

//    double all_start = std::clock();

//    std::cout << "bug test 1  cloud size is : " << input_cloud.size() << std::endl;
    for (int i = 0; i < input_cloud.size(); ++i) {

        pcl::PointXYZ tmp = input_cloud.points[i];

        if(tmp.x < 15.0 && tmp.x > -15) {
            if (tmp.y < 15.0 && tmp.y > -15) {
                float dist = sqrtf(tmp.x * tmp.x + tmp.y * tmp.y);
                if(dist > 4.0) {
                    cv::Point2i tmpIdx = TransToInd(cv::Point2f(tmp.x, tmp.y));

                    if (tmpIdx.x >= 0 && tmpIdx.x < grid_width &&
                            tmpIdx.y >= 0 && tmpIdx.y < grid_height) {
                        grid_to_sort.at<uchar>(tmpIdx.y, tmpIdx.x) += dist * grid_cell_length;
                    }
                }
            }
        }
    }

    std::vector<mat_mem> mem_to_sort;

    for (int i = 0; i < grid_height; ++i) {
        for (int j = 0; j < grid_width; ++j) {
            if (grid_to_sort.at<uchar>(i, j) > 5){

                mat_mem member = {0};
                member.x = j;
                member.y = i;
                member.num = grid_to_sort.at<uchar>(i,j);

                mem_to_sort.push_back(member);
            }
        }
    }

    std::stable_sort(mem_to_sort.begin(), mem_to_sort.end(), CompareNum);
    std::vector<mat_mem>::iterator sort_iter = mem_to_sort.begin();
    int marker_id = 1;
    std::vector<cv::Point2f> gridmap;

    for (; sort_iter != mem_to_sort.end() ; ++sort_iter) {

            int x, y;
            x = (*sort_iter).x;
            y = (*sort_iter).y;

        if(grid_cell_picked.at<uchar>(y, x) !=1 && marker_id <= grid_map_feature_num) {

            grid_to_match.at<uchar>(x, y) = 16 - marker_id;

            visualization_msgs::Marker marker;
            marker.header.frame_id = pointcloud.header.frame_id;
            marker.header.stamp = pointcloud.header.stamp;

            marker.ns = "basic_shapes";
            marker.id = marker_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = (x - grid_width / 2) * grid_cell_length;
            marker.pose.position.y = (y - grid_height / 2) * grid_cell_length;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 4.0;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
//            marker.color.a = (float) (0.1 * marker_id);
            marker.color.a = 1.0f;

            marker.lifetime = ros::Duration();

            markers.markers.push_back(marker);
            marker_id++;

            for (int j = y - sparsity; j <= y + sparsity; ++j) {
                for (int k = x - sparsity; k <= x + sparsity; ++k) {

                    if (j >= 0 && j < grid_width && k >= 0 && k < grid_height)
                        grid_cell_picked.at<uchar>(j, k) = 1;
                }
            }

            cv::Point2f point = cv::Point2f((x - grid_width / 2) * grid_cell_length,
                                            (y - grid_height / 2) * grid_cell_length);
//            std::cout << frame_id << "  x: " << x << "  y: " << y << std::endl;
            gridmap.push_back(point);

        }

        if(marker_id == grid_map_feature_num)
            break;
    }

    grid_maps.push_back(gridmap);
    std::cout << "grid map " << frame_id << " has been saved" << std::endl;

    markers_pub.publish(markers);
//    double all_end = std::clock();
//    std::cout << "All programme costs : " << all_end - all_start << std::endl;
//    std::cout << grid_maps.at(frame_id).at(0) << std::endl;

    frame_id++;
}

void OdomHandler(nav_msgs::Odometry odom){

    global_map_time = odom.header.stamp.toSec();
    global_update_flag = true;
}

void CloudHandler(sensor_msgs::PointCloud2 cloud){

    cloud_to_match_time.at(local_match_pointer) = cloud;
    local_match_pointer = (local_match_pointer + 1) % local_match_length;


    std::vector<sensor_msgs::PointCloud2>::iterator cloud_iter = cloud_to_match_time.begin();

    for (; cloud_iter < cloud_to_match_time.begin() + local_match_length; ++cloud_iter) {

        double diff_time = fabs(cloud_iter->header.stamp.toSec() - global_map_time);
        if(diff_time == 0 && cloud_iter->data.size() != 0 && global_update_flag){
            PointCloudHandler(*cloud_iter);
            pcl::PointCloud<pcl::PointXYZI> cloud_to_save;
            pcl::fromROSMsg(*cloud_iter, cloud_to_save);
//            SaveToPCD(cloud_to_save , frame_id);
            global_update_flag = false;
            break;
        }
    }
}

void MatchingHandler(loam_velodyne::MatchedPair pair){

    std::cout << "Get Pair! Go to matching~" << std::endl;

    if(MatchCorrespondingGridMap(grid_maps.at(pair.id1 - 1), grid_maps.at(pair.id2))){

            matched_pub.publish(pair);
    }

}

void SaveLocalMapHandler(sensor_msgs::PointCloud2 cloud){

}

int main (int arga, char** argv){

    ros::init(arga, argv, "BuildOccupancyGridMap");
    ros::NodeHandle nh_B;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("read_config_path",read_config_path);

//    std::cout << read_config_path << std::endl;

    read_xml();

    cloud_to_match_time.resize(local_match_length);

    ros::Subscriber subFeaturePointCloud = nh_B.subscribe<sensor_msgs::PointCloud2>
            ("/featurePoints", 2, CloudHandler);

    ros::Subscriber subMappingOdom = nh_B.subscribe<nav_msgs::Odometry>
            ("/integrated_to_init", 5, OdomHandler);

    ros::Subscriber subSaveLocalMap = nh_B.subscribe<sensor_msgs::PointCloud2>
            ("/rslidar_points", 2 , SaveLocalMapHandler);

    markers_pub = nh_B.advertise<visualization_msgs::MarkerArray>
            ("/visualization_marker", 1);

    matched_pub = nh_B.advertise<loam_velodyne::MatchedPair>
            ("/Matched_grid", 2);

    ros::Subscriber subGridSearch = nh_B.subscribe<loam_velodyne::MatchedPair>
            ("/Grid_to_search", 2, MatchingHandler);

    ros::spin();

    return 0;
}