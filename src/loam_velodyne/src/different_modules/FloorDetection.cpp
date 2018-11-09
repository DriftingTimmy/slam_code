//
// Created by lx on 18-6-28.
//

#include "FloorDetection.h"

namespace LxSlam{

    pcl::PointCloud<pcl::PointXYZI> FloorDetection::filter_floor
            (sensor_msgs::PointCloud2ConstPtr& cloud_msg){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *cloud);

        if(cloud->empty()) {
            return;
        }

        // floor detection
        boost::optional<Eigen::Vector4f> floor = detect(cloud);

        // publish the detected floor coefficients
        float coeffs[4] = {0};

        if(floor) {
            for(int i=0; i<4; i++) {
                coeffs[i] = (*floor)[i];
            }
        }

    }

    boost::optional<Eigen::Vector4f> FloorDetection::detect
            (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
        Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
        tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

        // filtering before RANSAC (height and normal filtering)
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
        filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
        filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);

        if(use_normal_filtering) {
            filtered = normal_filtering(filtered);
        }

        pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

        // too few points for RANSAC
        if(filtered->size() < floor_pts_thresh) {
            return boost::none;
        }

        // RANSAC
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p
                (new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(filtered));

        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        ransac.getInliers(inliers->indices);

        // too few inliers
        if(inliers->indices.size() < floor_pts_thresh) {
            return boost::none;
        }

        // verticality check of the detected floor's normal
        Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);

        double dot = coeffs.head<3>().dot(reference.head<3>());
        if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
            // the normal is not vertical
            return boost::none;
        }

        // make the normal upward
        if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
            coeffs *= -1.0f;
        }

        return Eigen::Vector4f(coeffs);

    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr FloorDetection::plane_clip
            (const pcl::PointCloud<pcl::PointXYZI>::Ptr& src_cloud,
             const Eigen::Vector4f& plane, bool negative){
        pcl::PlaneClipper3D<pcl::PointXYZI> clipper(plane);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);

        clipper.clipPointCloud3D(*src_cloud, indices->indices);

        pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(indices);
        extract.setNegative(negative);
        extract.filter(*dst_cloud);

        return dst_cloud;

    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr FloorDetection::normal_filtering
            (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(10);
        ne.setViewPoint(0.0f, 0.0f, sensor_height);
        ne.compute(*normals);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        filtered->reserve(cloud->size());

        for (int i = 0; i < cloud->size(); i++) {
            float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
            if (std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
                filtered->push_back(cloud->at(i));
            }
        }

        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;

        return filtered;

    }
}
///namespace LxSlam