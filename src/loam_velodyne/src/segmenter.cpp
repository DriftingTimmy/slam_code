//
// Created by lx on 18-6-12.
//

#include "segmenter.h"

namespace LxSlam {

bool swap_if_gt(float& a, float& b) {
    if (a > b) {
        std::swap(a, b);
        return true;
    }
    return false;
}

float Segmenter::compute_dist_between_centroid(segment seg1, segment seg2) {

    float dist_x = seg1.centroid[0] - seg2.centroid[0];
    float dist_y = seg1.centroid[1] - seg2.centroid[1];
    float dist_z = seg1.centroid[2] - seg2.centroid[2];

    float dist = sqrtf(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);

    return dist;
}

pcl::PointCloud<pcl::PointXYZI> Segmenter::filterInputCloud() {

    std::cout << "Start the filter part" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_filter_ptr
            (new pcl::PointCloud<pcl::PointXYZI>(*input_cloud_));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr
            (new pcl::PointCloud<pcl::PointXYZI>());

    voxel_filter_.setInputCloud(cloud_to_filter_ptr);
    voxel_filter_.setLeafSize(voxel_filter_grid_size_, voxel_filter_grid_size_,
                    voxel_filter_grid_size_);
    voxel_filter_.filter(*cloud_filtered_ptr);

    for (pcl::PointXYZI point : *cloud_filtered_ptr) {
        if (point.z > -2 && point.z < 3.5) {
            if (std::hypotf(point.x , point.y) < 30) {
                filtered_cloud_->push_back(point);
            }
        }
    }
    ///Radius and height filters to limit the amount of the processing points

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr
            (new pcl::PointCloud<pcl::PointXYZI>(*filtered_cloud_));

    outrem_.setInputCloud(filtered_cloud_ptr);
    outrem_.setRadiusSearch(outlier_removal_radius_);
    outrem_.setMinNeighborsInRadius (outlier_removal_min_neighbour_);
    outrem_.filter (*filtered_cloud_);//执行条件滤波,在半径为0.5在此半径内必须要有30个邻居点，此点才会保存

    return *filtered_cloud_;
}
//TODO: Simplify the filter part cause the floor detection part has produced a good filter cloud

void Segmenter::segCloud(){

    std::cout << "Seg the cloud now" << std::endl;
    pcl::search::KdTree<pcl::PointXYZI> kd_tree;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr
            (new pcl::PointCloud<pcl::PointXYZI>(*filtered_cloud_));

    kd_tree.setInputCloud(filtered_cloud_ptr);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree_ptr
            (new pcl::search::KdTree<pcl::PointXYZI>(kd_tree));

    euclidean_cluster_extractor_.setClusterTolerance(euclidean_cluster_ClusterTolerance_);
    euclidean_cluster_extractor_.setMinClusterSize(euclidean_cluster_MinClusterSize_);
    euclidean_cluster_extractor_.setMaxClusterSize(euclidean_cluster_MaxClusterSize_);
    euclidean_cluster_extractor_.setSearchMethod(kd_tree_ptr);

    euclidean_cluster_extractor_.setInputCloud(filtered_cloud_ptr);
    euclidean_cluster_extractor_.extract(cluster_indices);

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
//                        std::cout << "Intensity for the publish segment is : "
//                                  << cloud_filtered_ptr->points[*pit].intensity << std::endl;
        }
        segment_id_++;
//                    std::cout << "Segment id for the publish segment is : " << segment_id << std::endl;

        Segmenter::segment seg_cloud;
        seg_cloud.centroid[0] = x_all / size;
        seg_cloud.centroid[1] = y_all / size;
        seg_cloud.centroid[2] = z_all / size;
        seg_cloud.segcloud = segmented_cluster;
        seg_cloud.size = size;
        seg_cloud.time_stamp = it->header.stamp;
        describe(seg_cloud);

        add_segment_to_map(seg_cloud);
    }
}

void Segmenter::describe(Segmenter::segment& segmented_cloud){

    std::cout << "start describe the segments !" << std::endl;

    const int kNPoints = segmented_cloud.size;
    pcl::PointCloud<pcl::PointXYZI> variances;

    for (size_t i = 0u; i < kNPoints; ++i) {
        variances.push_back(pcl::PointXYZI());
        variances.points[i].x = segmented_cloud.segcloud.points[i].x - segmented_cloud.centroid[0];
        variances.points[i].y = segmented_cloud.segcloud.points[i].y - segmented_cloud.centroid[1];
        variances.points[i].z = segmented_cloud.segcloud.points[i].z - segmented_cloud.centroid[2];
    }

    const std::vector<size_t> row_indices_to_access = {0,0,0,1,1,2};
    const std::vector<size_t> col_indices_to_access = {0,1,2,1,2,2};
    Eigen::Matrix3f covariance_matrix;
    for (size_t i = 0u; i < row_indices_to_access.size(); ++i) {
        const size_t row = row_indices_to_access[i];
        const size_t col = col_indices_to_access[i];
        double covariance = 0;
        for (size_t k = 0u; k < kNPoints; ++k) {
            covariance += variances.points[k].data[row] * variances.points[k].data[col];
        }
        covariance /= kNPoints;
        covariance_matrix(row,col) = covariance;
        covariance_matrix(col,row) = covariance;
    }

    constexpr bool compute_eigenvectors = false;
    Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(covariance_matrix, compute_eigenvectors);
    std::vector<float> eigenvalues(3, 0.0);
    eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
    eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
    eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();

    swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
    swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

    double sum_eigenvalues = eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
    double e1 = eigenvalues.at(0) / sum_eigenvalues;
    double e2 = eigenvalues.at(1) / sum_eigenvalues;
    double e3 = eigenvalues.at(2) / sum_eigenvalues;

    const double sum_of_eigenvalues = e1 + e2 + e3;
    constexpr double kOneThird = 1.0/3.0;

    const double kNormalizationPercentile = 1.0;

    const double kLinearityMax = 28890.9 * kNormalizationPercentile;
    const double kPlanarityMax = 95919.2 * kNormalizationPercentile;
    const double kScatteringMax = 124811 * kNormalizationPercentile;
    const double kOmnivarianceMax = 0.278636 * kNormalizationPercentile;
    const double kAnisotropyMax = 124810 * kNormalizationPercentile;
    const double kEigenEntropyMax = 0.956129 * kNormalizationPercentile;
    const double kChangeOfCurvatureMax = 0.99702 * kNormalizationPercentile;
    const double kNPointsMax = 13200 * kNormalizationPercentile;

    segmented_cloud.eigen_value_feature[0] = (e1 - e2) / e1 / kLinearityMax;///linearity
    segmented_cloud.eigen_value_feature[1] = (e2 - e3) / e1 / kPlanarityMax;///planarity
    segmented_cloud.eigen_value_feature[2] = e3 / e1 / kScatteringMax;///scattering
    segmented_cloud.eigen_value_feature[3] = std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax;///omnivariance
    segmented_cloud.eigen_value_feature[4] = (e1 - e3) / e1 / kAnisotropyMax;///anisotropy
    segmented_cloud.eigen_value_feature[5] = (e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)) / kEigenEntropyMax;///eigen_entropy
    segmented_cloud.eigen_value_feature[6] = e3 / sum_of_eigenvalues / kChangeOfCurvatureMax;///change_of_curvature

//    std::cout << "\033[1;31m ***********eigen_value_set are : \033[0m" << segmented_cloud.eigen_value_feature[0]
//              << segmented_cloud.eigen_value_feature[1] << segmented_cloud.eigen_value_feature[2]
//              << segmented_cloud.eigen_value_feature[3] << segmented_cloud.eigen_value_feature[4]
//              << segmented_cloud.eigen_value_feature[5] << segmented_cloud.eigen_value_feature[6] << std::endl;

}

void Segmenter::add_segment_to_map(segment& valid_segment){
    if(filter_nearest_segment(valid_segment)){
        segment_map_.push_back(std::make_shared(valid_segment));

        std::cout << "Valid segment added to map" << std::endl;
    }else{
        std::cout << "Invalid segment!!" << std::endl;
    }
}

void Segmenter::find_segment_candidates() {

}

bool Segmenter::filter_nearest_segment(segment segment_to_add){
    if(centroid_cloud_->size() < 10){

        std::cout << "Lack of segments in the map " << std::endl;
        return true;
    }else{

        pcl::PointXYZ centroid_to_add;
        centroid_to_add.x = segment_to_add.centroid[0];
        centroid_to_add.y = segment_to_add.centroid[1];
        centroid_to_add.z = segment_to_add.centroid[2];

        std::vector<int> centroid_nearest;
        std::vector<float> nearest_dist;

        if(centroids_tree_.radiusSearch(centroid_cloud_, centroid_nearest,
                                        nearest_dist, max_dist_filter_segment_) != 0){

        }else{
            centroid_cloud_->push_back(centroid_to_add);
        }
    }
}

}///namespace LxSlam