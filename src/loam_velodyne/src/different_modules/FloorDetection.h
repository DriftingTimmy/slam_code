//
// Created by lx on 18-6-28.
//

#ifndef PROJECT_FLOORDETECTION_H
#define PROJECT_FLOORDETECTION_H

#include <ros/ros.h>
#include <ros/time.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

namespace LxSlam {
    class FloorDetection {
    public:
        pcl::PointCloud<pcl::PointXYZI> filter_floor
                (sensor_msgs::PointCloud2ConstPtr& cloud_msg){};

        boost::optional<Eigen::Vector4f> detect
                (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){};

        pcl::PointCloud<pcl::PointXYZI>::Ptr plane_clip
                (const pcl::PointCloud<pcl::PointXYZI>::Ptr& src_cloud,
                 const Eigen::Vector4f& plane, bool negative){};

        pcl::PointCloud<pcl::PointXYZI>::Ptr normal_filtering
                (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){};

    private:
        double tilt_deg;
        double sensor_height;
        double height_clip_range;
        int floor_pts_thresh;
        double floor_normal_thresh;
        bool use_normal_filtering;
        double normal_filter_thresh;
    };
}///namespace LxSlam

#endif //PROJECT_FLOORDETECTION_H

///Reference code website: https://github.com/koide3/hdl_graph_slam/blob/master/apps/floor_detection_nodelet.cpp