//
// Created by lx on 18-4-17.
//
#include "front_end.h"

static bool is_pose_init = false;
static int markers_sum = 0;

segmatch seg;
graphOptmizer graph_optimizer;
std::unordered_map<Id, Eigen::Vector3f> trajectory_pose;

static pcl::PointCloud<pcl::PointXYZI> source_map_saver;
const bool is_build_mode = true;
int segment_id_classifier = 1;
std::vector<cv::Point2f> centroid_map;

float segmatch::calculateCentroidDis(segment& seg1, segment& seg2) {
    float diff_x = seg1.centroid[0] - seg2.centroid[0];
    float diff_y = seg1.centroid[1] - seg2.centroid[1];
    float diff_z = seg1.centroid[2] - seg2.centroid[2];

    float dist = sqrtf(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    return dist;
}

/**********************************用杆状物的二维匹配方法**********************************/

/*********************************用杆状物的二维匹配方法 END*******************************/

bool geometricConsistenceJudging(std::vector<segment>& segments_match_map,
                                 pcl::PointCloud<pcl::PointXYZ>& map_centroid_cloud){

    std::cout << "Using geometric consistency to match the point!" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_frame_ptr
            (new pcl::PointCloud<pcl::PointXYZ>(segment_cloud_frame));
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_map_ptr
            (new pcl::PointCloud<pcl::PointXYZ>(map_centroid_cloud));

    pcl::CorrespondencesPtr frame_map_corres(new pcl::Correspondences());
    std::vector<Eigen::Matrix4f,
            Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_matcher;
    gc_matcher.setGCSize(0.1);
    gc_matcher.setGCThreshold(8);

    gc_matcher.setInputCloud(centroid_frame_ptr);
    gc_matcher.setSceneCloud(centroid_map_ptr);
    gc_matcher.setModelSceneCorrespondences(frame_map_corres);

    gc_matcher.recognize(rototranslations, clustered_corrs);

    for (size_t i = 0; i < rototranslations.size(); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

        // 打印出相对于输入模型的旋转矩阵与平移矩阵
        Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

        printf("\n");
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        printf("\n");
        printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
    }

    return true;
}///using pcl geometric consistence to get the rotation and translation matrix
//TODO: 3d Hough matching method may be useful here

void compareGeometricConsistences(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ref_index,
                                  Eigen::Vector3f& search_pose){
    std::vector<int> centroid_ind_search;
    std::vector<float> dist_ind_search;
    pcl::PointXYZ pose_search_point;
    pose_search_point.x = search_pose[0];
    pose_search_point.y = search_pose[1];
    pose_search_point.z = search_pose[2];

    std::vector<segment> segments_match_map;//map中存储的对应搜索的segment形成的vector
    pcl::PointCloud<pcl::PointXYZ> map_centroid_cloud;

    if(centroid_tree.radiusSearch(pose_search_point, max_search_match_radius,
                                  centroid_ind_search, dist_ind_search) > 10){

        for(int i = 0; i < centroid_ind_search.size(); i++ ){
            map_centroid_cloud.points.emplace_back
                    ({cloud_ref_index->points.at(centroid_ind_search.at(i)).x,
                      cloud_ref_index->points.at(centroid_ind_search.at(i)).y,
                      cloud_ref_index->points.at(centroid_ind_search.at(i)).z});
        }

        for(int j = 0; j < centroid_ind_search.size(); j++ ){
            segments_match_map.push_back
                    (segment_list.at(cloud_ref_index->points.at(centroid_ind_search.at(j)).intensity));
        }
    }else{
        std::cout << "\033[1;31m There is not enough segments to match!! \033[0m" << std::endl;
    }

    if(geometricConsistenceJudging(segments_match_map, map_centroid_cloud)){
        trajectory_pose = graph_optimizer.updateGraph();
    }
}///利用pcl自己封装的函数对当前数据和地图数据的几何一致性进行判断确定对应的转换关系

///对于匹配上的pair找到对应帧之间的匹配关系，根据centroid来计算对应的旋转矩阵

void getCentroidCloudFromValidSegments(pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud_ref){
    if(!segment_list.empty()){
        for(auto valid_segment : segment_list){
            pcl::PointXYZ output_point;
            output_point.x = valid_segment.second.centroid[0];
            output_point.y = valid_segment.second.centroid[1];
            output_point.z = valid_segment.second.centroid[2];

            output_cloud->push_back(output_point);

            pcl::PointXYZI output_point_ref;
            output_point_ref.x = valid_segment.second.centroid[0];
            output_point_ref.y = valid_segment.second.centroid[1];
            output_point_ref.z = valid_segment.second.centroid[2];
            output_point_ref.intensity = valid_segment.first;

            output_cloud_ref->push_back(output_point_ref);
        }
    }
    output_cloud->width = 1;
    output_cloud->height = output_cloud->points.size();

    output_cloud_ref->width = 1;
    output_cloud_ref->height = output_cloud_ref->points.size();
}

void filterCorrespondingSegment(segment& filter_segment,
                                pcl::PointXYZI& centroid_point_with_ids){

    std::cout << "Search the corresponding segment to replace or delete !" << std::endl;
    segment compared_segment = segment_list.at((int)centroid_point_with_ids.intensity);

    if(filter_segment.size > compared_segment.size + 100 &&
       filter_segment.time_stamp > compared_segment.time_stamp){
        segment_list.at((int)centroid_point_with_ids.intensity) = filter_segment;
    }
}

void segmatch::filterNearestSegment(std::vector<segment>& segment_cloud) {

    std::cout << "begin the filtering step" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud_source_ptr
            (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud_with_id
            (new pcl::PointCloud<pcl::PointXYZI>());
    getCentroidCloudFromValidSegments(centroid_cloud_source_ptr, centroid_cloud_with_id);

    centroid_tree.setInputCloud(centroid_cloud_source_ptr);

    int remove_counter = 0;
    for(auto it = segment_cloud.begin(); it != segment_cloud.end(); it++){
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        pcl::PointXYZ searchPoint;
        searchPoint.x = it->centroid[0];
        searchPoint.y = it->centroid[1];
        searchPoint.z = it->centroid[2];

        if(centroid_tree.radiusSearch(searchPoint, max_search_radius,
                                      pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
            float min_dist = max_search_radius;
            int minimum_dist_id = 0;
            int i = 0;
            for( ; i != pointIdxRadiusSearch.size(); i++){
                if(pointRadiusSquaredDistance.at(i) < min_dist){
                    min_dist = pointRadiusSquaredDistance.at(i);
                    minimum_dist_id = i;
                }
            }
            if(min_dist < min_centroid_dist){
                filterCorrespondingSegment(*it, centroid_cloud_with_id->points.at(pointIdxRadiusSearch.at(i)));
                remove_counter++;
                segment_cloud.erase(it);
            }
        }
    }

    compareGeometricConsistences(centroid_cloud_with_id, segment_cloud.at(0).pose);
    addSegmentToSource(segment_cloud);
    std::cout << remove_counter << " segments have been filtered!!" << std::endl;
}///滤掉与source地图中过于靠近的segment保证不会出现重叠的情况
///根据特征值计算欧式距离，满足阈值限制就认为是匹配上的一系列segment

void segmatch::addSegmentToSource(std::vector<segment>& seg_to_add) {

    for (auto segment_candidate : seg_to_add){
        segment_Id++;
        segment_list.insert({segment_Id, segment_candidate});
    }
}

void SegcloudHandler(loam_velodyne::SegmentCloud input_cloud_msg){

//    std::cout << "\033[1;31m Handle the segment cloud!! \033[0m" << std::endl;
    segment segment_cloud;
//    PointCloud segment_without_i;
    segment_cloud.frame_id = input_cloud_msg.frame_id;
    segment_cloud.size = input_cloud_msg.size;
    segment_cloud.time_stamp = input_cloud_msg.time_stamp;

    segment_cloud.centroid[0] = input_cloud_msg.centroid[0];
    segment_cloud.centroid[1] = input_cloud_msg.centroid[1];
    segment_cloud.centroid[2] = input_cloud_msg.centroid[2];
    pcl::fromROSMsg(input_cloud_msg.segcloud, segment_cloud.segcloud);

    segment_cloud.eigen_value_feature[0] = input_cloud_msg.eigen_value_feature[0];
    segment_cloud.eigen_value_feature[1] = input_cloud_msg.eigen_value_feature[1];
    segment_cloud.eigen_value_feature[2] = input_cloud_msg.eigen_value_feature[2];
    segment_cloud.eigen_value_feature[3] = input_cloud_msg.eigen_value_feature[3];
    segment_cloud.eigen_value_feature[4] = input_cloud_msg.eigen_value_feature[4];
    segment_cloud.eigen_value_feature[5] = input_cloud_msg.eigen_value_feature[5];
    segment_cloud.eigen_value_feature[6] = input_cloud_msg.eigen_value_feature[6];

    segment_cloud.pose[0] = input_cloud_msg.estimate_pose[0];
    segment_cloud.pose[1] = input_cloud_msg.estimate_pose[1];
    segment_cloud.pose[2] = input_cloud_msg.estimate_pose[2];

    ///Making the pointcloud of the frame
    pcl::PointXYZ segment_point;
    segment_point.x = input_cloud_msg.centroid[0];
    segment_point.y = input_cloud_msg.centroid[1];
    segment_point.z = input_cloud_msg.centroid[2];

    segment_cloud_frame.push_back(segment_point);

    if(segment_cloud.frame_id == frame_id_last){
        segments_in_one_frame.push_back(segment_cloud);
    }else{
        Id cur_segment_id;
        cur_segment_id.counter = segment_cloud.frame_id;
        cur_segment_id.time_ns = segment_cloud.time_stamp;
        seg.filterNearestSegment(segments_in_one_frame);
        graph_optimizer.addVertexToGraph(segment_cloud.pose, segment_cloud.time_stamp);

        ///Add corresponding pose of these segments

        segments_in_one_frame.clear();
        segments_in_one_frame.push_back(segment_cloud);
        frame_id_last = segment_cloud.frame_id;
        pose_last_matrix = pose_to_matrix(segment_cloud.pose[0], segment_cloud.pose[1],
                                          0, 0, 0, segment_cloud.pose[2]);

        segment_cloud_frame.clear();
    }
}///处理自定义消息交给segmatch类来处理

int main(int argc, char** argv){
    ros::init(argc, argv, "segmentProcess");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber segment_cloud_sub = nh.subscribe<loam_velodyne::SegmentCloud>
            ("/segments_to_detect", 20, SegcloudHandler);
    std::cout << "\033[1;31m Start the segmatching node!!!!! \033[0m"<< std::endl;

    graph_optimizer.init();
    ///Initialize the graph optimizer

    ros::spin();
    if(!ros::ok()){
        if(is_build_mode){
            pcl::io::savePCDFile("/home/lx/LX_SLAM_ws/src/loam_velodyne/save_test_map/segment_source_map.pcd", source_map_saver);
            std::cout << "Map has been saved!!!!!!!!!!" << std::endl;
        }
    }
    return 0;
}