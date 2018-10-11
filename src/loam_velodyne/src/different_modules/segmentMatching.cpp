//
// Created by lx on 18-3-15.
//

#include "segmentMatching.h"
#include <loam_velodyne/SegmentCloud.h>
#include "common_function.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

pcl::KdTreeFLANN<pcl::PointXY> kdtree_map;
pcl::PointCloud<pcl::PointXY>::Ptr map_cloud_for_kdtree(new pcl::PointCloud<pcl::PointXY>());

static bool is_pose_init = false;
static int markers_sum = 0;

segmatch seg;

static pcl::PointCloud<pcl::PointXYZI> source_map_saver;
const bool is_build_mode = false;
int segment_id_classifier = 1;
std::vector<cv::Point2f> centroid_map;
ros::Publisher *transformed_pub = NULL;
ros::Publisher *pose_ekf_pub = NULL;
ros::Publisher *search_point_pub = NULL;

float segmatch::calculateCentroidDis(segment& seg1, segment& seg2) {
    float diff_x = seg1.centroid[0] - seg2.centroid[0];
    float diff_y = seg1.centroid[1] - seg2.centroid[1];
    float diff_z = seg1.centroid[2] - seg2.centroid[2];

    float dist = sqrtf(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    return dist;
}

/**********************************用杆状物的二维匹配方法**********************************/
bool sort_distance_lx_dist(distance_lx a, distance_lx b)
{
    return a.dis < b.dis;
}

float dist_between(cv::Point2f a, cv::Point2f b)
{
    float dist = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
    return dist;
}

bool sort_count(const match& a, const match& b)
{
    if (a.count != b.count)
        return a.count > b.count;
    else
        return a.dist < b.dist;
}

void segmatch::loadMap() {

    pcl::PointCloud<pcl::PointXYZI> source_map_from_pcd;
    pcl::io::loadPCDFile("/home/lx/catkin_seg/src/segmatch/laser_mapper/demonstration_files/robosense/target_0.pcd",
                         source_map_from_pcd);
    segment load_map_segment;
    int segment_size;
    float x_sum, y_sum, z_sum;
    for(auto point : source_map_from_pcd.points){
        if(point.intensity == segment_id_classifier){
            load_map_segment.segcloud.push_back(point);
//            if(segment_id_classifier <=1000){
//                segment_point_cloud_pcd_1.push_back(pcl::PointXYZ(point.x, point.y, point.z));
//            }else if(segment_id_classifier >=500 && segment_id_classifier <1500){
//                segment_point_cloud_pcd_2.push_back(pcl::PointXYZ(point.x, point.y, point.z));
//            }else if(segment_id_classifier >=1000 && segment_id_classifier <=2000){
//                segment_point_cloud_pcd_3.push_back(pcl::PointXYZ(point.x, point.y, point.z));
//            }else if(segment_id_classifier >=1500){
//                segment_point_cloud_pcd_4.push_back(pcl::PointXYZ(point.x, point.y, point.z));
//            }
            ///use for icp match method
            x_sum += point.x;
            y_sum += point.y;
            z_sum += point.z;
            segment_size++;
        }
        else if(point.intensity != segment_id_classifier){

//            std::cout << "\033[1;31m The 666 intensity of this point is : " << point.intensity << std::endl;
            load_map_segment.centroid[0] = x_sum / segment_size;
            load_map_segment.centroid[1] = y_sum / segment_size;
            load_map_segment.centroid[2] = z_sum / segment_size;
            load_map_segment.size = segment_size;
            source_segment_cloud_pcd_.insert({segment_id_classifier , load_map_segment});

            pcl::PointXY kd_point;
            kd_point.x = load_map_segment.centroid[0];
            kd_point.y = load_map_segment.centroid[1];
//
            map_cloud_for_kdtree->push_back(kd_point);

//            segment_point_cloud_pcd_ += load_map_segment.segcloud;
            centroid_map.push_back(cv::Point2f(load_map_segment.centroid[0], load_map_segment.centroid[1]));

            load_map_segment.segcloud.clear();
            load_map_segment.segcloud.push_back(point);
            segment_id_classifier = point.intensity;
            x_sum = point.x;
            y_sum = point.y;
            z_sum = point.z;
            segment_size = 1;
        }
    }

    kdtree_map.setInputCloud(map_cloud_for_kdtree);

//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_save_ptr(new pcl::PointCloud<pcl::PointXYZI>(source_map_from_pcd));
//    pcl::VoxelGrid<pcl::PointXYZI> sor;
//    sor.setInputCloud(cloud_to_save_ptr);
//    sor.setLeafSize(0.2, 0.2, 0.2);
//    sor.filter(source_map_from_pcd);

    std::cout << "\033[1;31m The source segment map has been loaded successful and the size is: \033[0m"
              << centroid_map.size() << std::endl;
    sensor_msgs::PointCloud2 background_map_msg;
    source_map_from_pcd.header.frame_id = "/camera_init";
    pcl::toROSMsg(source_map_from_pcd, background_map_msg);
    background_map_pub.publish(background_map_msg);
}

Eigen::Matrix4f segmatch::matchSegmentsIcp(pcl::PointCloud<pcl::PointXYZ> cloud_icp,
                                float x, float y, float yaw) {
    double icp_start = std::clock();

    Eigen::Matrix4f result = pose_to_matrix(x, y, 0, 0, 0, yaw);
    PointCloud::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud::Ptr framePtr(new pcl::PointCloud<Point>(cloud_icp));
    PointCloud::Ptr mapPtr(new pcl::PointCloud<Point>(segment_point_cloud_pcd_1));

//    PointCloud::Ptr framePtr(new pcl::PointCloud<Point>(segment_point_cloud_pcd_1));
//    PointCloud::Ptr mapPtr(new pcl::PointCloud<Point>(cloud_icp));

    Eigen::Matrix4f initial_guess = pose_to_matrix(x, y, 0, 0, 0, yaw);

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(0.1);
    icp.setMaximumIterations (300);

    icp.setInputSource(framePtr);
    icp.setInputTarget(mapPtr);
    icp.align (*output_cloud, Eigen::Matrix4f::Identity());

    result = icp.getFinalTransformation();
    std::cout << "\033[1;31m *********************ICP SCORE IS :  \033[0m" << icp.getFitnessScore() << std::endl;

    double icp_end = std::clock();
    std::cout << "ICP process costs : " << icp_end - icp_start << std::endl;
    return result;
}

void segmatch::matchCorrespondingFrames(std::vector<cv::Point2f> map1,
                               std::vector<cv::Point2f> map2, double time_stamp){
//    std::cout << "\033[1;32m Begin the matchcorrespondingframes step\033[0m" << std::endl;

    std::cout << "\033[1;32m Size of two maps: \033[0m" << map1.size() << "  " << map2.size() << std::endl;

    if(map1.size() >= 6 && map2.size() >= 10 ){
        double start_time = std::clock();

        std::vector<distance_lx> map1_distance_lx;
        auto it_ori = map1.begin();

        for (; it_ori != map1.end() - 1; ++it_ori)
        {
            auto it_ori2 = it_ori + 1;
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
        auto it_ori3 = map2.begin();
        for (; it_ori3 != map2.end() - 1; ++it_ori3)
        {
            auto it_ori4 = it_ori3 + 1;

            for (; it_ori4 != map2.end(); ++it_ori4)
            {
                auto dis_ori = sqrt(powf((*it_ori3).x - (*it_ori4).x, 2) +
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

        auto it_distance_lx1 = map1_distance_lx.begin();
        for (; it_distance_lx1 != map1_distance_lx.end(); ++it_distance_lx1)
        {
            float temp_distance_lx1 = (*it_distance_lx1).dis;

            float theta_it_distance_lx1 = atan2f((*it_distance_lx1).b.y - (*it_distance_lx1).a.y,
                                                 (*it_distance_lx1).b.x - (*it_distance_lx1).a.x);
            auto it_distance_lx2 = map2_distance_lx.begin();

            for (; it_distance_lx2 != map2_distance_lx.end(); ++it_distance_lx2){

                float temp_distance_lx2 = (*it_distance_lx2).dis;
                float theta_it_distance_lx2 = atan2f((*it_distance_lx2).b.y - (*it_distance_lx2).a.y,
                                                     (*it_distance_lx2).b.x - (*it_distance_lx2).a.x);
                float theta_it_distance_lx2_2 = atan2f((*it_distance_lx2).a.y - (*it_distance_lx2).b.y,
                                                       (*it_distance_lx2).a.x - (*it_distance_lx2).b.x);

                ///segment的中心位置匹配远没有杆状物匹配精确，所以这里将放大阈值限制，将0.2放宽至0.8m
                if(fabs(temp_distance_lx1 - temp_distance_lx2) < max_dist_between_poles){
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

                    auto it_trans_to = map2.begin();
                    for (; it_trans_to != map2.end(); ++it_trans_to)
                    {
                        float tmp_trans_to_ori_x = (*it_trans_to).x * cosf(diff_trans_theta1) -
                                                   (*it_trans_to).y * sinf(diff_trans_theta1) ;
                        float tmp_trans_to_ori_y = (*it_trans_to).y * cosf(diff_trans_theta1) +
                                                   (*it_trans_to).x * sinf(diff_trans_theta1) ;

                        cv::Point2f it_trans_to_global;
                        it_trans_to_global.x = tmp_trans_to_ori_x + x_offset;
                        it_trans_to_global.y = tmp_trans_to_ori_y + y_offset;
                        auto it_ori_1 = map1.begin();
                        for (; it_ori_1 != map1.end(); ++it_ori_1)
                        {
                            float dist = dist_between(it_trans_to_global, (*it_ori_1));
                            if (dist < 1)
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

                    auto it_trans_to2 = map2.begin();
                    for (; it_trans_to2 != map2.end(); ++it_trans_to2)
                    {
                        float tmp_trans_to_ori_x = (*it_trans_to2).x * cosf(diff_trans_theta2) -
                                                   (*it_trans_to2).y * sinf(diff_trans_theta2) ;
                        float tmp_trans_to_ori_y = (*it_trans_to2).y * cosf(diff_trans_theta2) +
                                                   (*it_trans_to2).x * sinf(diff_trans_theta2) ;

                        cv::Point2f it_trans_to_global;
                        it_trans_to_global.x = tmp_trans_to_ori_x + x_offset_2;
                        it_trans_to_global.y = tmp_trans_to_ori_y + y_offset_2;
                        auto it_ori_1 = map1.begin();
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
//    std::cout << "end the matchcorrespondingframes step" << std::endl;

//    std::cout << "bug test 4 " << std::endl;
        std::stable_sort(pairs_sample.begin(), pairs_sample.end(), sort_count);

        std::cout << "size is " << pairs_sample.size() << std::endl;
        ///修正当无法匹配到正常的约束时对应的不输出
        if(!pairs_sample.empty()){
            match the_one = (*pairs_sample.begin());

            float pairs_trans_theta = atan2f(the_one.b.y - the_one.a.y, the_one.b.x - the_one.a.x);
//    std::cout << "bug test3" << pairs_trans_theta << std::endl;

            float pairs_oir_theta = atan2f(the_one.B.y - the_one.A.y, the_one.B.x - the_one.A.x);
            // a-b逆时钟旋转角度到地图A-B,车的姿态也要加上这个旋转角度（逆时针）

            float pairs_diffs = atan2f(sinf(pairs_oir_theta - pairs_trans_theta),
                                       cosf(pairs_oir_theta - pairs_trans_theta));
            //    std::cout << std::endl << "the trans theta is " << pairs_diffs / 3.1415 * 180 << std::endl;

            float x0 = the_one.a.x * cosf(pairs_diffs) - the_one.a.y * sinf(pairs_diffs);
            float y0 = the_one.a.y * cosf(pairs_diffs) + the_one.a.x * sinf(pairs_diffs);

            std::cout << "Grid match counter : " << the_one.count << std::endl;

            double end_time = std::clock();

//TODO: Try to provide the rotate and translation estimation of the mathced two grid maps.
            if(the_one.count >= min_match_num){

                std::cout << "\033[1;31m The grid map has been matched!!!!!!! "
                          << x0 << "  " << y0 << "  " << end_time - start_time << std::endl;

                loam_velodyne::SegmentCloud pose_ekf_msg;
                pose_ekf_msg.centroid[0] = x0;
                pose_ekf_msg.centroid[1] = y0;
                pose_ekf_msg.centroid[2] = pairs_oir_theta;
                pose_ekf_msg.time_stamp = time_stamp;

                std::cout << "Time stamp out : " << std::fixed << pose_ekf_msg.time_stamp << std::endl;

//                if(x0 >= 0 && y0 >= 0)
//                    pose_ekf_pub->publish(pose_ekf_msg);
                ///Unable the odometryMainteinance
            }
            else{
                std::cout << "Failed to match the grid" << std::endl;
            }
        }
    }else{
        std::cout << "The detecting segments amount is hard to match!!" << std::endl;
    }
}
/***********************************用杆状物的二维匹配方法 END**********************************/

void segmatch::linkMatchesCentroid(std::vector<matchpair>& pairs) {
    std::vector<cv::Point2f> frame1;
    std::vector<cv::Point2f> frame2;
    for(auto pair : pairs){
        frame1.push_back(cv::Point2f(pair.center1[0],pair.center1[1]));
        frame2.push_back(cv::Point2f(pair.center2[0],pair.center2[1]));
    }
    matchCorrespondingFrames(frame1,frame2, 0.0);

}///对于匹配上的pair找到对应帧之间的匹配关系，根据centroid来计算对应的旋转矩阵

void segmatch::findMatchesInFrames(std::vector<segment> segments_in_frame){
//    std::cout << "begin frame matching step" << std::endl;
    if(!source_segment_cloud_.empty()){
        for(auto seg_in_source : source_segment_cloud_){
            std::vector<matchpair> matched_segments;
            for(auto seg_in_source_frame : seg_in_source.second){
                for (auto segment : segments_in_frame)
                    findMatchesByFeature(segment, seg_in_source_frame, matched_segments);
            }
            if(matched_segments.size() >= min_num_links){
                linkMatchesCentroid(matched_segments);
            }
        }
    }
}///找到每一帧与每一帧的匹配的segment对数

void segmatch::filterNearestSegment(segment& segment_cloud) {

//    std::cout << "begin the filtering step" << std::endl;
    bool segment_can_be_add = true;
//    int num_of_filtered_segment;

    for(auto segment_in_source : source_segment_cloud_){
        auto it = segment_in_source.second.begin();
        for (; it != segment_in_source.second.end() ; ++it) {
            if(calculateCentroidDis(*it, segment_cloud) < min_dis_between_centroid){
                if(fabs(segment_cloud.time_stamp - it->time_stamp) < min_time_diff){
                    segment_can_be_add = false;
                    break;
                }
            }
        }
    }
    if(segment_can_be_add){
        if (!frame_id_set){
            frame_id_last_ = segment_cloud.frame_id;
            segment_cloud_.push_back(segment_cloud);
            frame_id_set = true;
        }else{
            if(frame_id_last_ == segment_cloud.frame_id){
//                std::cout << "frame_id_last has been set true" << std::endl;
                segment_cloud_.push_back(segment_cloud);
            }else{
                if(is_build_mode){
                    addSegmentToSource(segment_cloud_);
                }
                findMatchesInFrames(segment_cloud_);
//                std::cout << "segment has been added to source" << std::endl;

                segment_cloud_.clear();
                segment_cloud_.push_back(segment_cloud);
            }
        }
    }
}///滤掉与source地图中过于靠近的segment保证不会出现重叠的情况

int segmatch::findNextId() {

    int id = 0;
    return id;
}

void segmatch::findMatchesByFeature(segment& seg1, segment& seg2,
                                         std::vector<matchpair>& matched_segments){
    double dist_between_features;
    dist_between_features =
            sqrt(pow(seg1.eigen_value_feature[0] - seg2.eigen_value_feature[0], 2)+
                 pow(seg1.eigen_value_feature[1] - seg2.eigen_value_feature[0], 2)+
                 pow(seg1.eigen_value_feature[2] - seg2.eigen_value_feature[0], 2)+
                 pow(seg1.eigen_value_feature[3] - seg2.eigen_value_feature[0], 2)+
                 pow(seg1.eigen_value_feature[4] - seg2.eigen_value_feature[0], 2)+
                 pow(seg1.eigen_value_feature[5] - seg2.eigen_value_feature[0], 2)+
                 pow(seg1.eigen_value_feature[6] - seg2.eigen_value_feature[0], 2));
//    std::cout << "\033[1;31m Distance between features is : \033[0m"
//              << dist_between_features << std::endl;

    if(dist_between_features < max_dis_between_features){
        matchpair pair;
        pair.id1 = seg1.frame_id;
        pair.id2 = seg2.frame_id;
        pair.center1[0] = seg1.centroid[0];
        pair.center1[1] = seg1.centroid[1];
        pair.center1[2] = seg1.centroid[2];
        pair.center2[0] = seg2.centroid[0];
        pair.center2[1] = seg2.centroid[1];
        pair.center2[2] = seg2.centroid[2];
        pair.score = dist_between_features;
//        std::cout << "\033[1;31m Matched segments found! Score is : \033[0m"
//                  << pair.score << std::endl;

        matched_segments.push_back(pair);
    }
}///根据7维特征值计算欧式距离，满足阈值限制就认为是匹配上的一系列segment

void segmatch::addSegmentToSource(std::vector<segment>& seg_to_add) {

//    int id = findNextId();
    int id = seg_to_add.at(0).frame_id;
    source_segment_cloud_.insert({id, seg_to_add});

//    std::cout << "\033[1;31m the size of the source_segment_cloud is : \033[0m"
//              << source_segment_cloud_.size() << std::endl;
    for(auto segcloud_add_to_source : seg_to_add){
        source_cloud_map += segcloud_add_to_source.segcloud;
        source_map_saver += segcloud_add_to_source.segcloud;
    }
    sensor_msgs::PointCloud2 source_cloud_map_msg;
    pcl::toROSMsg(source_cloud_map, source_cloud_map_msg);
    source_cloud_map_msg.header.frame_id = "/camera_init";
    seg.source_seg_map_pub.publish(source_cloud_map_msg);

//    *source_map_pointer = source_cloud_map;
}

void SegcloudHandler(loam_velodyne::SegmentCloud input_cloud_msg){

//    std::cout << "\033[1;31m Handle the segment cloud!! \033[0m" << std::endl;
    segment segment_cloud;
//    PointCloud segment_without_i;
    segment_cloud.frame_id = input_cloud_msg.frame_id;
    segment_cloud.size = input_cloud_msg.size;
    segment_cloud.time_stamp = input_cloud_msg.time_stamp;

    visualization_msgs::MarkerArray markers;

//    pcl::fromROSMsg(input_cloud_msg.segcloud, segment_cloud.segcloud);
//    pcl::fromROSMsg(input_cloud_msg.segcloud, segment_without_i);

    segment_cloud.centroid[0] = input_cloud_msg.centroid[0];
    segment_cloud.centroid[1] = input_cloud_msg.centroid[1];
    segment_cloud.centroid[2] = input_cloud_msg.centroid[2];

    pose_estimate[0] = input_cloud_msg.estimate_pose[0];
    pose_estimate[1] = input_cloud_msg.estimate_pose[1];
    pose_estimate[2] = input_cloud_msg.estimate_pose[2];

    float inc_x = input_cloud_msg.inc_dist[0];
    float inc_y = input_cloud_msg.inc_dist[1];
    float diff_theta = input_cloud_msg.inc_dist[2];
    float inc_distance = input_cloud_msg.inc_dist[3];
    float diff_time = input_cloud_msg.diff_time;

    if(is_build_mode){
        segment_cloud.eigen_value_feature[0] = input_cloud_msg.eigen_value_feature[0];
        segment_cloud.eigen_value_feature[1] = input_cloud_msg.eigen_value_feature[1];
        segment_cloud.eigen_value_feature[2] = input_cloud_msg.eigen_value_feature[2];
        segment_cloud.eigen_value_feature[3] = input_cloud_msg.eigen_value_feature[3];
        segment_cloud.eigen_value_feature[4] = input_cloud_msg.eigen_value_feature[4];
        segment_cloud.eigen_value_feature[5] = input_cloud_msg.eigen_value_feature[5];
        segment_cloud.eigen_value_feature[6] = input_cloud_msg.eigen_value_feature[6];

        seg.filterNearestSegment(segment_cloud);
    }else{
        if(!id_set){
            id_last = segment_cloud.frame_id;
//            point_cloud_icp += segment_without_i;
            centroid_frame.push_back(cv::Point2f(segment_cloud.centroid[0],segment_cloud.centroid[1]));
            id_set = true;
        }else{
            if(segment_cloud.frame_id == id_last){

//                std::cout << "Id Last is " << id_last << std::endl;

                centroid_frame.push_back(cv::Point2f(segment_cloud.centroid[0],segment_cloud.centroid[1]));
//                point_cloud_icp += segment_without_i;
///Unable ICP
            }else{
                std::cout << "\033[1;31m The size of segments!!!!!!!!!!!!! \033[0m "<< centroid_frame.size() << std::endl;

                /***********************************Poles frame match*******************************************/
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
//
                pcl::PointXY point_to_search;

                if(!is_pose_init){
                    point_to_search.x = pose_estimate[0];
                    point_to_search.y = pose_estimate[1];

                    is_pose_init = true;
                }else{
                    point_to_search.x = ( pose_estimate[0] + pose_estimate_last[0] ) * 0.5;
                    point_to_search.y = ( pose_estimate[1] + pose_estimate_last[1] ) * 0.5;
                }

                std::cout << "Search point pose is : " << point_to_search << std::endl;

                if(kdtree_map.radiusSearch(point_to_search, max_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){

                    std::cout << "Find corresponding local poles frames!  " << pointIdxRadiusSearch.size() << std::endl;
                    size_t i = 0;
                    std::vector<cv::Point2f> local_search_map;
                    pcl::PointCloud<pcl::PointXY> search_centroids;
                    int marker_id = 0;
                    while(i < pointIdxRadiusSearch.size()){
                        cv::Point2f point_selected;
                        point_selected.x = map_cloud_for_kdtree->points[pointIdxRadiusSearch[i]].x;
                        point_selected.y = map_cloud_for_kdtree->points[pointIdxRadiusSearch[i]].y;
                        search_centroids.push_back(map_cloud_for_kdtree->points[pointIdxRadiusSearch[i]]);

                        local_search_map.push_back(point_selected);

///Markers to show the candidates segments

                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "/camera_init";
//                        marker.header.stamp = time_stamp;

                        marker.ns = "basic_shapes";
                        marker.id = marker_id;
                        marker.type = visualization_msgs::Marker::CUBE;
                        marker.action = visualization_msgs::Marker::ADD;

                        marker.pose.position.x = map_cloud_for_kdtree->points[pointIdxRadiusSearch[i]].x;
                        marker.pose.position.y = map_cloud_for_kdtree->points[pointIdxRadiusSearch[i]].y;
                        marker.pose.position.z = -10;
                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;

                        marker.scale.x = 0.5;
                        marker.scale.y = 0.5;
                        marker.scale.z = 4.0;

                        marker.color.r = 0.0f;
                        marker.color.g = 1.0f;
                        marker.color.b = 0.0f;
//            marker.color.a = (float) (0.1 * marker_id);
                        marker.color.a = 1.0f;

                        marker.lifetime = ros::Duration();

                        markers.markers.push_back(marker);
                        marker_id++;
///
                        i++;
                    }

                    int segment_marker_id = marker_id;
                    auto iter = centroid_frame.begin();
                    while(iter != centroid_frame.end()){

                        visualization_msgs::Marker seg_marker;
                        seg_marker.header.frame_id = "/camera_init";
//                        marker.header.stamp = time_stamp;

                        seg_marker.ns = "basic_shapes";
                        seg_marker.id = segment_marker_id;
                        seg_marker.type = visualization_msgs::Marker::CUBE;
                        seg_marker.action = visualization_msgs::Marker::ADD;

                        seg_marker.pose.position.x = iter->x;
                        seg_marker.pose.position.y = iter->y;
                        seg_marker.pose.position.z = 0;
                        seg_marker.pose.orientation.x = 0.0;
                        seg_marker.pose.orientation.y = 0.0;
                        seg_marker.pose.orientation.z = 0.0;
                        seg_marker.pose.orientation.w = 1.0;

                        seg_marker.scale.x = 0.5;
                        seg_marker.scale.y = 0.5;
                        seg_marker.scale.z = 4.0;

                        seg_marker.color.r = 1.0f;
                        seg_marker.color.g = 0.0f;
                        seg_marker.color.b = 0.0f;
//            marker.color.a = (float) (0.1 * marker_id);
                        seg_marker.color.a = 1.0f;

                        seg_marker.lifetime = ros::Duration();

                        markers.markers.push_back(seg_marker);
                        segment_marker_id++;
                        iter++;
                    }

                    while(segment_marker_id < markers_sum){

                        visualization_msgs::Marker delete_marker;
                        delete_marker.header.frame_id = "/camera_init";

                        delete_marker.ns = "basic_shapes";
                        delete_marker.id = segment_marker_id;
                        delete_marker.type = visualization_msgs::Marker::CUBE;
                        delete_marker.action = visualization_msgs::Marker::DELETE;

                        markers.markers.push_back(delete_marker);

                        segment_marker_id++;
                    }

                    search_point_pub->publish(markers);
                    markers.markers.clear();
                    markers_sum = centroid_frame.size() + pointIdxRadiusSearch.size();

//                    search_centroids.header.frame_id = "/camera_init";
//                    sensor_msgs::PointCloud2 search_centroids_msg;
//                    pcl::toROSMsg(search_centroids,search_centroids_msg);
//                    search_point_pub->publish(search_centroids_msg);
                    std::cout << "Make the local map to match! " << std::endl;
                    seg.matchCorrespondingFrames(centroid_frame, local_search_map, input_cloud_msg.time_stamp);
                }

                /***********************************Poles frame match end*******************************************/

//                seg.matchCorrespondingFrames(centroid_frame, centroid_map);
//                Eigen::Matrix4f relative_pose = seg.matchSegmentsIcp(point_cloud_icp, input_cloud_msg.estimate_pose[0],
//                                     input_cloud_msg.estimate_pose[1], input_cloud_msg.estimate_pose[2]);
//
//                Eigen::Matrix4f global_pose = relative_pose * pose_to_matrix(pose_estimate[0], pose_estimate[1], 0,
//                                                                             0, 0, pose_estimate[2]);
//
//                double x, y, z, roll, pitch, yaw;
//                matrix_to_pose(global_pose, x, y, z, roll, pitch, yaw);
//                measure_pose[0] = x;
//                measure_pose[1] = y;
//                measure_pose[2] = yaw;
//
//                /*******************************EKF Part*********************************/
//
////                std::cout << "Begin the EKF part" << std::endl;
//
//                Af << 1, 0, -inc_distance * sinf(pose_estimate[2] + diff_theta/2.0),
//                        0, 1,  inc_distance * cosf(pose_estimate[2] + diff_theta/2.0),
//                        0, 0, 1;
//
//                Q << 2 * diff_time, 0, 0,
//                        0, 2 * diff_time, 0,
//                        0, 0, 2 * diff_time;
//
//                R << 5, 0, 0,
//                        0, 5, 0,
//                        0, 0, 0.5;
////                std::cout << "Inc distance is : " << inc_distance << std::endl;
//
//                pred_true_Covariance = Af * esti_true_Covariance * Af.transpose() + Q;
//
//                predict_pose[0] = pose_estimate[0];
//                predict_pose[1] = pose_estimate[1];
//                predict_pose[2] = pose_estimate[2];
//
////                std::cout << "Diff time is : " << diff_time << std::endl;
//
//                k_filter = pred_true_Covariance * (pred_true_Covariance + R).inverse();
//                Eigen::Vector3f diff_pose1;
//                diff_pose1 = measure_pose - predict_pose;
//                diff_pose1[2] = atan2f(sinf(diff_pose1[2]), cosf(diff_pose1[2]));
//                estimate_pose = predict_pose + k_filter * diff_pose1;
//
//                float mea_yaw = measure_pose[2];
//                float pre_yaw = predict_pose[2];
//                float k_yaw = k_filter(2, 2);
//                estimate_pose[2] = atan2f(k_yaw * sinf(mea_yaw) + (1 - k_yaw) * sinf(pre_yaw),
//                                          k_yaw * cosf(mea_yaw) + (1 - k_yaw) * cosf(pre_yaw));
//                esti_true_Covariance = (Eigen::Matrix3f::Identity() - k_filter) * pred_true_Covariance;
//
//                std::cout << "EKF is end! Relative_pose is : \n" << relative_pose << std::endl;
//
//                std::cout << "EKF is end! Estimate_pose is : \n" << global_pose << std::endl;
//
//                /*******************************EKF END*********************************/
//
//                relative_pose = relative_pose.inverse();
//                pcl::transformPointCloud(point_cloud_icp, point_cloud_icp, relative_pose);
//                sensor_msgs::PointCloud2 trans_pointcloud_msg;
//                point_cloud_icp.header.frame_id = "/camera_init";
//                pcl::toROSMsg(point_cloud_icp, trans_pointcloud_msg);
//                transformed_pub->publish(trans_pointcloud_msg);
//
////                pcl::PointCloud<pcl::PointXYZ> pose_ekf_cloud(2, 1);
////                pose_ekf_cloud.points[0].x = estimate_pose[0];
////                pose_ekf_cloud.points[0].y = estimate_pose[1];
////                pose_ekf_cloud.points[0].z = estimate_pose[2];
////                pose_ekf_cloud.points[1].x = input_cloud_msg.time_stamp;
//
////                std::cout << "Time stamp out : " << std::fixed << pose_ekf_cloud.points[1].x << std::endl;
//
////                sensor_msgs::PointCloud2 pose_ekf_msg;
////                pcl::toROSMsg(pose_ekf_cloud, pose_ekf_msg);
//
//                loam_velodyne::SegmentCloud pose_ekf_msg;
//                pose_ekf_msg.centroid[0] = estimate_pose[0];
//                pose_ekf_msg.centroid[1] = estimate_pose[1];
//                pose_ekf_msg.centroid[2] = estimate_pose[2];
//                pose_ekf_msg.time_stamp = input_cloud_msg.time_stamp;
//
//                std::cout << "Time stamp out : " << std::fixed << pose_ekf_msg.time_stamp << std::endl;
//
//
//                pose_ekf_pub->publish(pose_ekf_msg);
//

                pose_estimate_last[0] = pose_estimate[0];
                pose_estimate_last[1] = pose_estimate[1];

                id_last = segment_cloud.frame_id;
//                centroid_map.clear();
                centroid_frame.clear();
//                point_cloud_icp.clear();

            }
        }
    }
}///处理自定义消息交给segmatch类来处理

int main(int argc, char** argv){
    ros::init(argc, argv, "segmentMatching");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    seg.source_seg_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/source_segment_representation", 5);
    background_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/background", 5);

    ros::Publisher points_transformed = nh.advertise<sensor_msgs::PointCloud2>("/points_tranformed", 10);
    transformed_pub = &points_transformed;

    ros::Publisher pose_ekf = nh.advertise<loam_velodyne::SegmentCloud>("/pose_ekf", 1);
    pose_ekf_pub = &pose_ekf;

//    ros::Publisher search_pose = nh.advertise<visualization_msgs::MarkerArray>("/search_point_centroids", 10);
//    search_point_pub = &search_pose;

    ros::Publisher makers_pub = nh.advertise<visualization_msgs::MarkerArray>("/search_point_centroids", 100);
    search_point_pub = &makers_pub;

    ros::Subscriber segment_cloud_sub = nh.subscribe<loam_velodyne::SegmentCloud>("/segments_to_detect", 20, SegcloudHandler);

//    ros::Subscriber segment_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/seg_points", 5, SegcloudHandler);

    if(!is_build_mode){
        seg.loadMap();
    }

//    std::cout << "\033[1;31m Start the segmatching node!!!!! \033[0m"<< std::endl;

    ros::spin();
    if(!ros::ok()){
        if(is_build_mode){
            pcl::io::savePCDFile("/home/lx/LX_SLAM_ws/src/loam_velodyne/save_test_map/segment_source_map.pcd",source_map_saver);
            std::cout << "Map has been saved!!!!!!!!!!" << std::endl;
        }
    }
    return 0;
}