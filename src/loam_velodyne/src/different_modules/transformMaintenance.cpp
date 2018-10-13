#include <cmath>
#include "tinyxml.h"

//#include <loam_velodyne/common.h>
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

using namespace std;
typedef __SIZE_TYPE__ size_t;

float transformSum[6] = {0};
float transformIncre[6] = {0};
float transformMapped[6] = {0};
float transformMappedLast[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

ros::Publisher *pubLaserOdometry2Pointer = NULL;
nav_msgs::Odometry laserOdometry2;
tf::StampedTransform laserOdometryTrans2;

void transformAssociateToMap()
{
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

    float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                             - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                             - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
    transformMapped[0] = -asin(srx);

    float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                         - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                   - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                                + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                   + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
    float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                         - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                   + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                                + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                   - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                                + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
    transformMapped[1] = atan2(srycrx / cos(transformMapped[0]),
                               crycrx / cos(transformMapped[0]));

    float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                                 - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                   - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                                   - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                   + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                                 - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                   - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                                   - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                   + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
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

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;

    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;

    transformAssociateToMap();

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


}

void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
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

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transformMaintenance");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");


    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
            ("/laser_odom_to_init", 5, laserOdometryHandler);

    ros::Subscriber subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>
            ("/aft_mapped_to_init", 5, odomAftMappedHandler);

    ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry>
            ("/integrated_to_init", 5);

    pubLaserOdometry2Pointer = &pubLaserOdometry2;
    laserOdometry2.header.frame_id = "/camera_init";
    laserOdometry2.child_frame_id = "/camera";

    laserOdometryTrans2.frame_id_ = "/camera_init";
    laserOdometryTrans2.child_frame_id_ = "/camera";

    ros::spin();

    return 0;
}


Eigen::Vector3f getEKFPose(Eigen::Vector3f& predict_pose, Eigen::Vector3f& measure_pose,
                           float& diff_theta, double& diff_time, float& inc_dist){
    std::cout << "Begin output the EKF pose !" << std::endl;

    Eigen::Vector3f ekf_pose;

    Af << 1, 0, -inc_dist * sinf(predict_pose[2] + diff_theta/2.0),
            0, 1,  inc_dist * cosf(predict_pose[2] + diff_theta/2.0),
            0, 0, 1;

    Q << 2 * diff_time, 0, 0,
            0, 2 * diff_time, 0,
            0, 0, 2 * diff_time;

    R << 5, 0, 0,
            0, 5, 0,
            0, 0, 0.5;

    pred_true_Covariance = Af * esti_true_Covariance * Af.transpose() + Q;

    k_filter = pred_true_Covariance * (pred_true_Covariance + R).inverse();
    Eigen::Vector3f diff_pose1;
    diff_pose1 = measure_pose - predict_pose;
    diff_pose1[2] = atan2f(sinf(diff_pose1[2]), cosf(diff_pose1[2]));
    ekf_pose = predict_pose + k_filter * diff_pose1;
    float mea_yaw = measure_pose[2];
    float pre_yaw = predict_pose[2];
    float k_yaw = k_filter(2, 2);
    ekf_pose[2] = atan2f(k_yaw * sinf(mea_yaw) + (1 - k_yaw) * sinf(pre_yaw),
                         k_yaw * cosf(mea_yaw) + (1 - k_yaw) * cosf(pre_yaw));
    esti_true_Covariance = (Eigen::Matrix3f::Identity() - k_filter) * pred_true_Covariance;

    return ekf_pose;
}

void segmentCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr,
                  Eigen::Vector3f pose){

    double start = std::clock();

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean_cluster_extractor;
    pcl::search::KdTree<pcl::PointXYZI> kd_tree;
    std::vector<pcl::PointIndices> cluster_indices;

    kd_tree.setInputCloud(cloud_filtered_ptr);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree_ptr(new pcl::search::KdTree<pcl::PointXYZI>(kd_tree));
    euclidean_cluster_extractor.setClusterTolerance(0.4);//ec_tolerance is 0.2 if default
    euclidean_cluster_extractor.setMinClusterSize(300);//max_size is 15000 if default
    euclidean_cluster_extractor.setMaxClusterSize(15000);//min_size is 100 if default
    euclidean_cluster_extractor.setSearchMethod(kd_tree_ptr);

    euclidean_cluster_extractor.setInputCloud(cloud_filtered_ptr);
    euclidean_cluster_extractor.extract(cluster_indices);

//                int j = 0;
//                std::vector<segment> segmented_cloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZI> segmented_cluster;
        float x_all = 0;
        float y_all = 0;
        float z_all = 0;
        int size = 0;

        for (auto pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_filtered_ptr->points[*pit].intensity = segment_id;
            cloud_cluster->points.push_back(cloud_filtered_ptr->points[*pit]);
            segmented_cluster.push_back(cloud_filtered_ptr->points[*pit]);
            x_all += cloud_filtered_ptr->points[*pit].x;
            y_all += cloud_filtered_ptr->points[*pit].y;
            z_all += cloud_filtered_ptr->points[*pit].z;
            size++;
//                        std::cout << "Intensity for the publish segment is : "
//                                  << cloud_filtered_ptr->points[*pit].intensity << std::endl;
        }
        segment_id++;
//                    std::cout << "Segment id for the publish segment is : " << segment_id << std::endl;

        segment seg_cloud;
        seg_cloud.centroid[0] = x_all/size;
        seg_cloud.centroid[1] = y_all/size;
        seg_cloud.centroid[2] = z_all/size;
        seg_cloud.segcloud = segmented_cluster;
        seg_cloud.size = size;
        seg_cloud.frame_id = frame_id;
        seg_cloud.time_stamp = it->header.stamp;
///Unable the description part of the segment
//                    DescribeSegmentedCloud(seg_cloud);

        loam_velodyne::SegmentCloud segment_msg;
        segment_msg.centroid[0] = seg_cloud.centroid[0];
        segment_msg.centroid[1] = seg_cloud.centroid[1];
        segment_msg.centroid[2] = seg_cloud.centroid[2];
        pcl::toROSMsg(seg_cloud.segcloud, segment_msg.segcloud);

        for (int i = 0; i < 7; ++i) {
            segment_msg.eigen_value_feature[i] = seg_cloud.eigen_value_feature[i];
        }
        segment_msg.size = seg_cloud.size;
        segment_msg.frame_id = seg_cloud.frame_id;
        segment_msg.time_stamp = seg_cloud.time_stamp;
        segment_msg.estimate_pose[0] = pose[0];
        segment_msg.estimate_pose[1] = pose[1];
        segment_msg.estimate_pose[2] = pose[2];


        segment_pub->publish(segment_msg);
//                    j++;
//                    std::cout << "segment has been published!" << std::endl;
//                    segmented_cloud.push_back(seg_cloud);

    }///put all clusters together
//
//            cloud_cluster->width = cloud_cluster->points.size();
//            cloud_cluster->height = 1;

    cloud_cluster->is_dense = true;
    cloud_cluster->header.stamp = cur_time;
    sensor_msgs::PointCloud2 cloud_cluster_pub;
    pcl::toROSMsg(*cloud_cluster, cloud_cluster_pub);
//                std::cout << "The clustered cloud size is : " << cloud_cluster_pub.data.size() << std::endl;
//    cloud_cluster_pub.header.stamp = cloud_iter->header.stamp;
    cloud_cluster_pub.header.frame_id = "/camera_init";
    seg_points_pub->publish(cloud_cluster_pub);

    double end = std::clock();
    std::cout << "segment costs : " << end - start << std::endl;
}

void DescribeSegmentedCloud(segment& segmented_cloud){

//    cout << "start describe the segments !" << endl;

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


//struct FeatureValue {
//    FeatureValue(std::string feature_name, double feature_value) :
//            name(feature_name), value(feature_value) {}
//    std::string name = "";
//    double value = 0.0;
//};

Eigen::Vector3f mergeMultiSensors(double diff_time, float x, float y, float yaw,
                                  float &inc_distance, float &diff_theta) {
    std::cout << "Begin merge the sensors' messages!" << std::endl;

    inc_distance = (cur_vel + last_vel) / 2 * diff_time;
    diff_theta = cur_theta - last_theta;

    Eigen::Vector3f inc_distance_car;
    inc_distance_car << inc_distance * cosf(diff_theta / 2.), inc_distance * sinf(diff_theta / 2.), diff_theta;

    Eigen::Matrix3f car_rotate;
    car_rotate << cosf(yaw), -sinf(yaw), 0,
            sinf(yaw), cosf(yaw), 0,
            0, 0, 1;

    Eigen::Vector3f inc_distance_global;
    Eigen::Vector3f integrated_pose = {x, y, yaw};
    inc_distance_global = car_rotate * inc_distance_car;
    integrated_pose += inc_distance_global;

    return integrated_pose;
}

//                    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
//                    outrem.setInputCloud(cloud_filtered_ptr);
//                    outrem.setRadiusSearch(0.5);     //设置半径为0.5的范围内找临近点
//                    outrem.setMinNeighborsInRadius(30); //设置查询点的邻域点集数小于30的删除
//                    outrem.filter(*cloud_filtered_ptr);     //执行条件滤波,在半径为0.5在此半径内必须要有30个邻居点，此点才会保存

//            sensor_msgs::PointCloud2 cloud_cluster_pub;
//            pcl::toROSMsg(cloud_mutual, cloud_cluster_pub);
//
//            std::cout << "The clustered cloud size is : " << cloud_cluster_pub.data.size() << std::endl;
//            cloud_cluster_pub.header.stamp = cloud_iter->header.stamp;
//            cloud_cluster_pub.header.frame_id = "/camera_init";
//            seg_points_pub->publish(cloud_cluster_pub);
///Test the single frame
//            Eigen::Matrix4f transform = pose_to_matrix(transformMapped[3],transformMapped[4],
//                                                       transformMapped[5],transformMapped[1],
//                                                       -transformMapped[0],transformMapped[2]);


//                Eigen::Vector3f measure_pose = {transformMapped[3], transformMapped[4], -transformMapped[1]};
//                Eigen::Vector3f ekf_pose = getEKFPose(predict_pose, measure_pose, diff_theta, diff_time, inc_dist);
//                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_save_ptr(
//                        new pcl::PointCloud<pcl::PointXYZI>(cloud_to_save));

//                float inc_dist;
//                float diff_theta;
//            Eigen::Vector3f predict_pose = mergeMultiSensors(diff_time, transformMappedLast[3],
//                                                             transformMappedLast[4], -transformMappedLast[1],
//                                                             inc_dist, diff_theta);

//            std::cout << "%%%%The output odometry is " << transformMapped[0] << " " << transformMapped[1] << " " << transformMapped[2]
//            <<" " << transformMapped[3]<<" " << transformMapped[4]<<" " << transformMapped[5] <<std::endl;

//            posefile << frame_id << " " << transformMapped[3] << " " << transformMapped[4] << " " << transformMapped[5]
//                     << " " << transformMapped[2] << " " << -transformMapped[0] << " " << -transformMapped[1]
//                     << " " << score << std::endl;