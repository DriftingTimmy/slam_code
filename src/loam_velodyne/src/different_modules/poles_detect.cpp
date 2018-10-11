//
// Created by lx on 17-12-29.
//

#include <math.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <loam_velodyne/KeyFrame.h>
#include <loam_velodyne/MatchedPair.h>
#include "subMap.h"
#include "common_function.h"
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>


using namespace std;
float length = 0.3;//每个栅格的边长
int min_density = 200;//候选网格中最少点数
int min_points = 400;//ransac后，内点最少数

class xyz {
public:
    float x;
    float y;
    float z;
    xyz(float a, float b, float c) {
        x = a;
        y = b;
        z = c;
    }
};
class mesh
{
public:
    float high = -100;
    float low = 100;
    int density = 0;//存储网格密度，这里是该网格中点的数量
    int state = 0;//0表示不是候选区（圆柱体处），1表示是
    vector<xyz> point;
    mesh() {}
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "poles_detect");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lx/Rosbags/bg.pcd", *cloud);
    size_t len = cloud->points.size();
    //计算包围盒(之后看看能否用pcl优化)
    //下面这个函数就是求极值的，先不改了，而且不知道为什么计算行列时，用min_x等会报错
    //pcl::PointXYZ minPt, maxPt;
    //pcl::getMinMax3D(*cloud, minPt, maxPt);
    float min_x = 10000.0;
    float min_y = 10000.0;
    double max_x = -10000.0;
    double max_y = -10000.0;
    for (int i = 0; i < len; i++) {
        if (cloud->points[i].x < min_x)
            min_x = cloud->points[i].x;
        if (cloud->points[i].y < min_y)
            min_y = cloud->points[i].y;
        if (cloud->points[i].x > max_x)
            max_x = cloud->points[i].x;
        if (cloud->points[i].y > max_y)
            max_y = cloud->points[i].y;
    }
    int row = floor(10.596/0.3);// floor((max_x - min_x) / length);//,列的数量减1，一行有所少个减1
    int column = floor(34.4328/0.3);// floor((max_y - min_y) / length);//行的数量减1，一列有所少个减1
    vector< vector<mesh> > allmesh(column + 1);//外部是行，内部是列
    for (int i = 0; i < column + 1; i++) {
        allmesh[i].resize(row + 1);
    }
    //将每个点放入对应的栅格中,并且得到high和low以及density值
    for (int i = 0; i < len; i++) {
        //floor()是向负无穷方向取整,ceil()是向正无穷方向取整,还有fix(),round()
        int m = floor((cloud->points[i].x - min_x) / length);//该点应该在栅格的第m列,初始为第0列
        int n = floor((cloud->points[i].y - min_y) / length);//该点应该在栅格的第n行
        xyz xyz1 = xyz(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        allmesh[n][m].point.push_back(xyz1);//第n行第m列
        if (allmesh[n][m].high<cloud->points[i].z)
            allmesh[n][m].high = cloud->points[i].z;
        if (allmesh[n][m].low>cloud->points[i].z)
            allmesh[n][m].low = cloud->points[i].z;
        allmesh[n][m].density++;
    }
    //直接找出所有密度大于阈值的栅格，缺点1，阈值不好设定；缺点2，一个树干可能造成好几个高密度的栅格
    for (int i = 0; i <= column; i++) {
        for (int j = 0; j <= row; j++) {
            if (allmesh[i][j].density > min_density) {
                allmesh[i][j].state = 1;//标记为候选栅格
            }
        }
    }
    //以候选栅格为中心，取3*3的栅格中的点为输入点云，并将此区域内的所有栅格的stata置为1
    //暂时不计算在边上的栅格
    std::cout << "begin1" << std::endl;

    int number_candidate = 0;
    for (int i = 1; i < column; i++) {
        for (int j = 1; j < row; j++) {
            //将候选区域的点都放在candidate中,并输出
            if (allmesh[i][j].state == 1) {
                vector<xyz> candidate;
                for (int a = -1; a < 2; a++) {
                    for (int b = -1; b < 2; b++) {
                        for (int m = 0; m < allmesh[i + a][j + b].point.size(); m++) {
                            xyz xyz2 = xyz(allmesh[i + a][j + b].point[m].x, allmesh[i + a][j + b].point[m].y, allmesh[i + a][j + b].point[m].z);
                            candidate.push_back(xyz2);
                        }
                        allmesh[i + a][j + b].state = 0;//周围八个栅格的state置为0
                    }
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZ>);//ransac处理的点云
                int leng = candidate.size();
                candidate_cloud->points.resize(leng);
                for (int c = 0; c < leng; c++) {
                    candidate_cloud->points[c].x = candidate[c].x;
                    candidate_cloud->points[c].y = candidate[c].y;
                    candidate_cloud->points[c].z = candidate[c].z;
                }

                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;//计算点云中所有点的法向量
                pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;//实例化分割
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//计算邻域的方法
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);//存储法线
                pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
                pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
                pcl::PCDWriter writer;

                std::cout << "begin2" << std::endl;

                ne.setSearchMethod(tree);
                ne.setInputCloud(candidate_cloud);
                ne.setKSearch(10);
                ne.compute(*cloud_normals);

                seg.setOptimizeCoefficients(true);//是否优化
                seg.setModelType(pcl::SACMODEL_CYLINDER);//模型
                seg.setMethodType(pcl::SAC_RANSAC);//方式
                seg.setAxis({0,0,1});//z轴方向
                seg.setEpsAngle(0.08);//偏离角度（弧度制）
                seg.setMaxIterations(1000);//最大迭代次数
                seg.setNormalDistanceWeight(0.2);
                seg.setDistanceThreshold(0.1);//判断内外点的距离阈值
                seg.setRadiusLimits(0, 0.5);//圆柱最大圆半径为0.5米
                seg.setInputCloud(candidate_cloud);
                seg.setInputNormals(cloud_normals);
                seg.segment(*inliers_cylinder, *coefficients_cylinder);

                extract.setInputCloud(candidate_cloud);
                extract.setIndices(inliers_cylinder);
                extract.setNegative(false);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
                extract.filter(*cloud_cylinder);

                char filename[256] = { 0 };
                char indexStr[16] = { 0 };
                strcat(filename, "yuanzhuti_");
                sprintf(indexStr, "%d", number_candidate);
                strcat(filename, indexStr);

                if (cloud_cylinder->points.size() > min_points) {
                    number_candidate++;
                    writer.write(filename, *cloud_cylinder, false);
                }
            }
        }
    }
}
