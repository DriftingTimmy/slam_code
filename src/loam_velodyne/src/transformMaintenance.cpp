#include <cmath>
#include "tinyxml.h"

#include <loam_velodyne/common.h>
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

//typedef pcl::PointXYZ Point;
//typedef pcl::PointXYZI PointI;
//typedef pcl::PointCloud<Point> PointCloud;
//typedef pcl::PointCloud<PointI> PointICloud;
//typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
//typedef pcl::PointCloud<PointI>::Ptr PointICloudPtr;

using namespace std;
namespace LxSlam {
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

    double cur_time = 0, cur_vel = 0, cur_theta = 0;
    double last_time = 0, last_vel = 0, last_theta = 0;
    const float dist_to_build_local_map = 5;
    nav_msgs::Odometry obd_msg;
    sensor_msgs::Imu Imu_msg;
    Odometry odom;
    Segmenter segmenter;

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

    void read_xml() {

        TiXmlDocument doc;
        if (!doc.LoadFile(read_config_path)) {
            std::cout << "error_xmltrans" << std::endl;
        } else {
            std::cout << "read_xml" << std::endl;
        }

        TiXmlElement *node = doc.FirstChildElement("lx_mapping");

        TiXmlElement *local_map_save_path_Elem = node->FirstChildElement("local_map_save_path");

        local_map_save_path = local_map_save_path_Elem->GetText();

//    save_local_path = local_map_save_path;
        save_pose = local_map_save_path;
        save_pose.append("mapping_pose.txt");
        posefile.open(save_pose);

//    std::cout << "xml has been load!" << std::endl;
    }

    bool swap_if_gt(float &a, float &b) {
        if (a > b) {
            std::swap(a, b);
            return true;
        }
        return false;
    }

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

    Eigen::Matrix4f pose_to_matrix(const double &pose_x, const double &pose_y, const double &pose_z,
                                   const double &pose_roll, const double &pose_pitch, const double &pose_yaw) {
        ///generate transform matrix according to current pose
        Eigen::AngleAxisf current_rotation_x(pose_roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf current_rotation_y(pose_pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf current_rotation_z(pose_yaw, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f current_translation(pose_x, pose_y, pose_z);
        Eigen::Matrix4f transform_matrix =
                (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

        return transform_matrix;
    }

    bool check_translation() {

        float diff_x = transformMapped[3] - transformMappedLast[3];
        float diff_y = transformMapped[4] - transformMappedLast[4];
//    float diff_z = transformMapped[5] - transformMappedLast[5];

        float dist = sqrtf(diff_x * diff_x + diff_y * diff_y);
        if (dist >= 0.08)
            return true;
        else
            return false;

    }

    void SaveToPCD(pcl::PointCloud<pcl::PointXYZI> cloud, int id) {

        stringstream ss;
        save_local_path = local_map_save_path;
        ss << id;
        string frame_num = ss.str();
        frame_num.append("laser.pcd");
        save_local_path.append(frame_num);
        std::vector<int> indicies;
        pcl::removeNaNFromPointCloud(cloud, cloud, indicies);

        pcl::io::savePCDFileASCII(save_local_path, cloud);
        std::cout << "local_laser has been saved!!" << std::endl;
    }

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

/*************************************************************************************************************************/

        auto cloud_iter = cloud_to_match.begin();
        for (; cloud_iter < cloud_to_match.begin() + 10; ++cloud_iter) {

            double check_time = fabs(cloud_iter->header.stamp.toSec() - laserOdometry->header.stamp.toSec());
            if (check_time == 0 && !cloud_iter->data.empty() && check_translation()) {
                ///Choose the corresponding pointcloud of the same timestamp
                ///Begin EKF part
                double diff_time = cur_time - last_time;
                float inc_dist;
                float diff_theta;
//            Eigen::Vector3f predict_pose = mergeMultiSensors(diff_time, transformMappedLast[3],
//                                                             transformMappedLast[4], -transformMappedLast[1],
//                                                             inc_dist, diff_theta);

                odom.MergeSensorData(Imu_msg, obd_msg, diff_time);
//                Eigen::Vector3f measure_pose = {transformMapped[3], transformMapped[4], -transformMapped[1]};
//
//                Eigen::Vector3f ekf_pose = getEKFPose(predict_pose, measure_pose, diff_theta, diff_time, inc_dist);
                Eigen::Matrix4f ekf_pose = odom.get_ekf_pose();
                ///Begin the filter part
                pcl::PointCloud<pcl::PointXYZI> cloud_to_save;
                pcl::fromROSMsg(*cloud_iter, cloud_to_save);

//                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_save_ptr(
//                        new pcl::PointCloud<pcl::PointXYZI>(cloud_to_save));
//                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());
//                pcl::VoxelGrid<pcl::PointXYZI> sor;
//                sor.setInputCloud(cloud_to_save_ptr);
//                sor.setLeafSize(0.2, 0.2, 0.2);
//                sor.filter(*cloud_filtered_ptr);
//
////            auto size = cloud_filtered_ptr->size();
//                pcl::PointCloud<pcl::PointXYZI> cloud_mutual;
//                for (pcl::PointXYZI point : *cloud_filtered_ptr) {
//                    if (point.z > -1.5 && point.z < 3.5) {
//                        if (std::hypotf(point.x, point.y) < 30) {
//                            cloud_mutual.push_back(point);
//                        }
//                    }
//                }
                segmenter.setInputCloud(cloud_to_save);
                segmenter.filterInputCloud();
                segmenter.segCloud();
                ///Radius and height filters to limit the amount of the processing points

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
                ///Eigen::Matrix4f transform = pose_to_matrix(ekf_pose[0], ekf_pose[1], 0, 0, 0, ekf_pose[2]);
                Eigen::Matrix4f transform = ekf_pose;
                ///对应方式不是上面的getQuat函数里面的对应关系
                pcl::PointCloud<pcl::PointXYZI> cloud_to_add;
                pcl::transformPointCloud(cloud_mutual, cloud_to_add, transform);
//            std::cout << "%%%%The output odometry is " << transformMapped[0] << " " << transformMapped[1] << " " << transformMapped[2]
//            <<" " << transformMapped[3]<<" " << transformMapped[4]<<" " << transformMapped[5] <<std::endl;

                if (0.2 < std::hypotf(fabs(transformMapped[3] - transformMappedLast[3]),
                                      fabs(transformMapped[4] - transformMappedLast[4])) < dist_to_build_local_map ||
                    !aft_mapped_pose_updated) {
                    cloud_for_cluster += cloud_to_add;
                    break;
                }///Add several frames together to make the clusters' pointcloud dense.
                else {
                    ///SaveToPCD(cloud_to_save , frame_id);
//            posefile << frame_id << " " << transformMapped[3] << " " << transformMapped[4] << " " << transformMapped[5]
//                     << " " << transformMapped[2] << " " << -transformMapped[0] << " " << -transformMapped[1]
//                     << " " << score << std::endl;
                    ///save the pose of the single frame
                    *cloud_filtered_ptr = cloud_for_cluster;

                    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
                    outrem.setInputCloud(cloud_filtered_ptr);
                    outrem.setRadiusSearch(0.5);     //设置半径为0.5的范围内找临近点
                    outrem.setMinNeighborsInRadius(30); //设置查询点的邻域点集数小于30的删除
                    outrem.filter(*cloud_filtered_ptr);     //执行条件滤波,在半径为0.5在此半径内必须要有30个邻居点，此点才会保存

                    /******************************segment************************************/
                    segmentCloud(cloud_filtered_ptr, ekf_pose);
                    /******************************segment************************************/
//            std::cout << posefile << std::endl;

//            Odometry_seg.stamp_ = cloud_iter->header.stamp;
//            Odometry_seg.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
//            Odometry_seg.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
//            tf_seg->sendTransform(Odometry_seg);
//            seg_points_pub->publish(*cloud_iter);
                    ///publish pointcloud for connecting segmatch algorithm.

                    frame_id++;
                    transformMappedLast[0] = transformMapped[0];
                    transformMappedLast[1] = transformMapped[1];
                    transformMappedLast[2] = transformMapped[2];
                    transformMappedLast[3] = transformMapped[3];
                    transformMappedLast[4] = transformMapped[4];
                    transformMappedLast[5] = transformMapped[5];

                    last_time = cur_time;
                    last_theta = cur_theta;
                    last_vel = cur_vel;

                    cloud_for_cluster.clear();
                    break;
                }
            }
        }

        laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        tfBroadcaster2Pointer->sendTransform(laserOdometryTrans2);

        last_time = cur_time;

/***********************************************************************************************************************/
    }

    void obdCallback(const nav_msgs::OdometryConstPtr &vel_msg) {
        obd_msg = *vel_msg;
    }

    void imuCallback(sensor_msgs::Imu imu_msg) {
//    geometry_msgs::Quaternion q = imu_msg.orientation;
//    cur_theta = -tf::getYaw(q);
        Imu_msg = imu_msg;

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

        private_nh.getParam("read_config_path", read_config_path);
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

        ros::Subscriber obd_sub = nh.subscribe<nav_msgs::Odometry>("/canbus/canbus", 10, obdCallback);

        ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/nav440/nav440", 10, imuCallback);

        ros::Publisher seg_points_pub2 = nh.advertise<sensor_msgs::PointCloud2>("/seg_points", 5);
        seg_points_pub = &seg_points_pub2;

//    ros::Publisher segment_pub2 = nh.advertise<loam_velodyne::SegmentCloud>("/segments_to_detect", 5);
//    segment_pub = &segment_pub2;

        tf::TransformBroadcaster tf_seg2;
        tf_seg = &tf_seg2;
        Odometry_seg.frame_id_ = "/rs_odom";
        Odometry_seg.child_frame_id_ = "/base_link2";
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
}