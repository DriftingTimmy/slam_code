//
// Created by timmy on 17-11-28.
//

#ifndef PROJECT_COMMON_FUNCTION_H
#define PROJECT_COMMON_FUNCTION_H
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
//#include <pcl/filters/voxel_grid.h>

#include <sys/types.h>
#include <sys/stat.h>
#include "tinyxml.h"

struct Pose{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

Eigen::Matrix4f pose_to_matrix(const double & pose_x,    const double & pose_y,     const double & pose_z,
                               const double & pose_roll, const double & pose_pitch, const double & pose_yaw);

Eigen::Matrix4f
pose_to_matrix(const double &pose_x, const double &pose_y, const double &pose_z, const double &pose_roll,
               const double &pose_pitch, const double &pose_yaw) {
    ///generate transform matrix according to current pose
    Eigen::AngleAxisf current_rotation_x( pose_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf current_rotation_y( pose_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf current_rotation_z( pose_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f current_translation(pose_x, pose_y, pose_z);
    Eigen::Matrix4f transform_matrix =
            (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

    return transform_matrix ;
}

void matrix_to_pose(Eigen::Matrix4f & transform_matrix, double & pose_x, double & pose_y, double & pose_z,
                    double & pose_roll, double & pose_pitch, double & pose_yaw)
{
    ///get current pose according to ndt transform matrix
    tf::Matrix3x3 mat_rotate;
    mat_rotate.setValue(static_cast<double>(transform_matrix(0, 0)), static_cast<double>(transform_matrix(0, 1)),
                        static_cast<double>(transform_matrix(0, 2)), static_cast<double>(transform_matrix(1, 0)),
                        static_cast<double>(transform_matrix(1, 1)), static_cast<double>(transform_matrix(1, 2)),
                        static_cast<double>(transform_matrix(2, 0)), static_cast<double>(transform_matrix(2, 1)),
                        static_cast<double>(transform_matrix(2, 2)));

    pose_x = transform_matrix(0, 3);
    pose_y = transform_matrix(1, 3);
    pose_z = transform_matrix(2, 3);
    mat_rotate.getRPY(pose_roll, pose_pitch, pose_yaw, 1);
}

void Isometry_to_Matrix(Eigen::Isometry3d & input, Eigen::Matrix4f & output)
{
    output(0,0) =  input.matrix()(0,0); output(0,1) =  input.matrix()(0,1);
    output(0,2) =  input.matrix()(0,2); output(0,3) =  input.matrix()(0,3);
    output(1,0) =  input.matrix()(1,0); output(1,1) =  input.matrix()(1,1);
    output(1,2) =  input.matrix()(1,2); output(1,3) =  input.matrix()(1,3);
    output(2,0) =  input.matrix()(2,0); output(2,1) =  input.matrix()(2,1);
    output(2,2) =  input.matrix()(2,2); output(2,3) =  input.matrix()(2,3);
    output(3,0) =  input.matrix()(3,0); output(3,1) =  input.matrix()(3,1);
    output(3,2) =  input.matrix()(3,2); output(3,3) =  input.matrix()(3,3);
}

void Matrix_to_Isometry(Eigen::Matrix4f & input,Eigen::Isometry3d & output)
{
    output(0,0) =  input(0,0); output(0,1) =  input(0,1);
    output(0,2) =  input(0,2); output(0,3) =  input(0,3);
    output(1,0) =  input(1,0); output(1,1) =  input(1,1);
    output(1,2) =  input(1,2); output(1,3) =  input(1,3);
    output(2,0) =  input(2,0); output(2,1) =  input(2,1);
    output(2,2) =  input(2,2); output(2,3) =  input(2,3);
    output(3,0) =  input(3,0); output(3,1) =  input(3,1);
    output(3,2) =  input(3,2); output(3,3) =  input(3,3);
}

//void voxel_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & input_cloud,
//                  pcl::PointCloud<pcl::PointXYZI>::Ptr & output_cloud,const float & voxel_leaf_size)
//{
//    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
//    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
//    voxel_grid_filter.setInputCloud(input_cloud);
//    voxel_grid_filter.filter(*output_cloud);
//}

std::fstream& go_to_line(std::fstream &file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(unsigned int p=0; p < num - 1; ++p)
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

std::vector<float> get_txt_data(std::fstream &file, unsigned int line, const unsigned int data_num)
{
    go_to_line(file, line);
    std::string str;
    std::getline(file, str);
    std::vector<float> data;
    std::istringstream iss(str);

    for (unsigned int count=0; count<data_num; count++)
    {
        std::string sub;
        iss >> sub;
        double value = ::atof(sub.c_str());
        data.push_back(value);
    }
    return data;
}
#endif //PROJECT_COMMON_FUNCTION_H
