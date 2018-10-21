//
// Created by lx on 18-6-12.
//

#ifndef PROJECT_SEGMENTER_H
#define PROJECT_SEGMENTER_H

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace LxSlam{

class Segmenter {

public:

    struct segment{
        int size;
        int frame_id;
        float centroid[3] = {0};
        double time_stamp;
        double eigen_value_feature[7] = {0};
        Eigen::Vector3f pose;
        pcl::PointCloud<pcl::PointXYZI> segcloud;
    };

    struct Segfactor{
        double  segment_descriptor[7] = {0};
        float   segment_centroid[3]   = {0};
    };

    void setInputCloud(pcl::PointCloud<pcl::PointXYZI> cloud){ *input_cloud_ = cloud; };

    pcl::PointCloud<pcl::PointXYZI> filterInputCloud(){};

    void segCloud(){};

    bool find_segment_candidates(){};


private:
    const int euclidean_cluster_MinClusterSize_     = 300;//min_size is 100 if defalt
    const int euclidean_cluster_MaxClusterSize_     = 15000;//max_size is 15000 if default
    const int outlier_removal_min_neighbour_        = 30;//设置查询点的邻域点集数小于30的删除

    const float euclidean_cluster_ClusterTolerance_ = 0.4;//ec_tolerance is 0.2 if default
    const float voxel_filter_grid_size_             = 0.2;
    const float outlier_removal_radius_             = 0.5;//设置半径为0.5的范围内找临近点

    const float max_dist_filter_segment_            = 0.3;

    void describe(segment& segmented_cloud){};
    void add_segment_to_map(segment& valid_segment){};
    float compute_dist_between_centroid(segment seg1, segment seg2){};
    bool filter_nearest_segment(segment segment_to_add){};
    void descriptors_match(){};
    void compare_segment(){};

    int segment_id_ = 0;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean_cluster_extractor_;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> input_cloud_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> filtered_cloud_;

    std::vector<std::shared_ptr<Segfactor>> segment_descriptor_map_;
    std::vector<std::shared_ptr<segment>> segment_map_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> centroid_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> centroids_tree_;

};

}///namespace LxSlam

#endif //PROJECT_SEGMENTER_H
