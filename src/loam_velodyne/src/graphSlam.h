//
// Created by lx on 18-6-15.
//

#ifndef PROJECT_GRAPHSLAM_H
#define PROJECT_GRAPHSLAM_H
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <common_function.h>

struct pairs
{
    cv::Point2f pole;
    cv::Point2f map_pole;
    float distance;
};

struct double_pairs
{
    cv::Point2f xa_1;
    cv::Point2f XA_1;
    cv::Point2f xa_2;
    cv::Point2f XA_2;
    float theta;
};

struct distance
{
    cv::Point2f a;
    cv::Point2f b;
    float dis;
};

struct match
{
    cv::Point2f A;
    cv::Point2f B;
    cv::Point2f a;
    cv::Point2f b;
    float dist;
    int count;
};

struct single_tree
{
    cv::Point2f location;
    float height;
    int count;
};

bool sort_distance(const pairs& a, const pairs& b)
{
    return a.distance < b.distance;
}

bool sort_count(const match& a, const match& b)
{
    if (a.count != b.count)
        return a.count > b.count;
    else
        return a.dist < b.dist;
}

bool sort_distance_dist(const distance& a, const distance& b)
{
    return a.dis < b.dis;
}

float dist_between(cv::Point2f a, cv::Point2f b)
{
    float dist = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
    return dist;
}

int Transform(std::vector<cv::Point2f> trans_points, Eigen::Vector3f estimate_pose,
                                  Eigen::Vector3f& rotated_pose, std::vector<cv::Point2f> ori_points)
{

//    kdTreeRadiusSearch(estimate_pose, 50, ori_points);

    std::vector<distance> ori_distance;
    std::vector<cv::Point2f>::iterator it_ori = ori_points.begin();
    for (; it_ori != ori_points.end() - 1; ++it_ori)
    {
        std::vector<cv::Point2f>::iterator it_ori2 = it_ori + 1;
        for (; it_ori2 != ori_points.end(); ++it_ori2)
        {
            float dis_ori = sqrt(powf((*it_ori).x - (*it_ori2).x, 2) + powf((*it_ori).y - (*it_ori2).y, 2));
            distance temp_distance;
            temp_distance.a = (*it_ori);
            temp_distance.b = (*it_ori2);
            temp_distance.dis = dis_ori;
            ori_distance.push_back(temp_distance);
        }
    }

//    std::vector<cv::Point2f> trans_points;
//    std::vector<cv::Point3f>::iterator it_poles = poles_points.begin();
//    for (; it_poles != poles_points.end(); ++it_poles)
//    {
//        if ((*it_poles).z < 2.4)
//            continue;
//        float tran_x = atan2f(sinf(estimate_pose[2]), cosf(estimate_pose[2]));
//        float trans_xa = (*it_poles).x * cosf(tran_x) - (*it_poles).y * sinf(tran_x);
//        float trans_ya = (*it_poles).y * cosf(tran_x) + (*it_poles).x * sinf(tran_x);
//        float global_xa = trans_xa + estimate_pose[0];
//        float global_ya = trans_ya + estimate_pose[1];
//        trans_points.push_back(cv::Point2f(global_xa, global_ya));
//    }

    std::vector<distance> trans_distance;
    std::vector<cv::Point2f>::iterator it_trans = trans_points.begin();
    for (; it_trans != trans_points.end() - 1; ++it_trans)
    {
        std::vector<cv::Point2f>::iterator it_trans2 = it_trans + 1;
        for (; it_trans2 != trans_points.end(); ++it_trans2)
        {
            float dis_trans = sqrt(powf((*it_trans).x - (*it_trans2).x, 2) + powf((*it_trans).y - (*it_trans2).y, 2));
            distance temp_distance;
            temp_distance.a = (*it_trans);
            temp_distance.b = (*it_trans2);
            temp_distance.dis = dis_trans;
            trans_distance.push_back(temp_distance);
        }
    }

    std::stable_sort(ori_distance.begin(), ori_distance.end(), sort_distance_dist);
    std::stable_sort(trans_distance.begin(), trans_distance.end(), sort_distance_dist);
    std::vector<cv::Point2f> ori_points_2 = ori_points;
    float car_x = estimate_pose[0];
    float car_y = estimate_pose[1];

    std::vector<match> pairs_sample;

    std::vector<distance>::iterator it_ori_distance = ori_distance.begin();
    for (; it_ori_distance != ori_distance.end(); ++it_ori_distance)
    {
        float temp_ori_distance = (*it_ori_distance).dis;

        float theta_it_ori_distance =
                atan2f((*it_ori_distance).b.y - (*it_ori_distance).a.y, (*it_ori_distance).b.x - (*it_ori_distance).a.x);
        std::vector<distance>::iterator it_trans_distance = trans_distance.begin();
        for (; it_trans_distance != trans_distance.end(); ++it_trans_distance)
        {
            float temp_trans_distance = (*it_trans_distance).dis;
            float theta_it_trans_distance = atan2f((*it_trans_distance).b.y - (*it_trans_distance).a.y,
                                                   (*it_trans_distance).b.x - (*it_trans_distance).a.x);
            float theta_it_trans_distance_2 = atan2f((*it_trans_distance).a.y - (*it_trans_distance).b.y,
                                                     (*it_trans_distance).a.x - (*it_trans_distance).b.x);
            if (fabs(temp_ori_distance - temp_trans_distance) < 0.2)
            {
                match pairs_a_a, pairs_a_b;
                // a对a，b对b
                pairs_a_a.A = (*it_ori_distance).a;
                pairs_a_a.B = (*it_ori_distance).b;
                pairs_a_a.a = (*it_trans_distance).a;
                pairs_a_a.b = (*it_trans_distance).b;
                pairs_a_a.dist = fabs(temp_trans_distance - temp_ori_distance);
                pairs_a_a.count = 0;
                float diff_trans_to_ori_theta = atan2f(sinf(theta_it_ori_distance - theta_it_trans_distance),
                                                       cosf(theta_it_ori_distance - theta_it_trans_distance));
                float a_rotate_x = pairs_a_a.a.x * cosf(diff_trans_to_ori_theta) -
                                   pairs_a_a.a.y * sinf(diff_trans_to_ori_theta) + car_x +
                                   car_y * sinf(diff_trans_to_ori_theta) - car_x * cosf(diff_trans_to_ori_theta);
                float a_rotate_y = pairs_a_a.a.y * cosf(diff_trans_to_ori_theta) +
                                   pairs_a_a.a.x * sinf(diff_trans_to_ori_theta) + car_y -
                                   car_y * cosf(diff_trans_to_ori_theta) - car_x * sinf(diff_trans_to_ori_theta);

                float x_offset = pairs_a_a.A.x - a_rotate_x;
                float y_offset = pairs_a_a.A.y - a_rotate_y;

                std::vector<cv::Point2f>::iterator it_trans_to = trans_points.begin();
                for (; it_trans_to != trans_points.end(); ++it_trans_to)
                {
                    float tmp_trans_to_ori_x = (*it_trans_to).x * cosf(diff_trans_to_ori_theta) -
                                               (*it_trans_to).y * sinf(diff_trans_to_ori_theta) + car_x +
                                               car_y * sinf(diff_trans_to_ori_theta) -
                                               car_x * cosf(diff_trans_to_ori_theta);
                    float tmp_trans_to_ori_y = (*it_trans_to).y * cosf(diff_trans_to_ori_theta) +
                                               (*it_trans_to).x * sinf(diff_trans_to_ori_theta) + car_y -
                                               car_y * cosf(diff_trans_to_ori_theta) -
                                               car_x * sinf(diff_trans_to_ori_theta);

                    cv::Point2f it_trans_to_global;
                    it_trans_to_global.x = tmp_trans_to_ori_x + x_offset;
                    it_trans_to_global.y = tmp_trans_to_ori_y + y_offset;
                    std::vector<cv::Point2f>::iterator it_ori_1 = ori_points_2.begin();
                    for (; it_ori_1 != ori_points_2.end(); ++it_ori_1)
                    {
                        float dist = dist_between(it_trans_to_global, (*it_ori_1));
                        if (dist < 0.15)
                        {
                            pairs_a_a.count++;
                            break;
                        }
                    }
                }
                pairs_a_b.A = (*it_ori_distance).a;
                pairs_a_b.B = (*it_ori_distance).b;
                pairs_a_b.a = (*it_trans_distance).b;
                pairs_a_b.b = (*it_trans_distance).a;
                pairs_a_b.dist = fabs(temp_trans_distance - temp_ori_distance);
                pairs_a_b.count = 0;
                float diff_trans_to_ori_theta_2 = atan2f(sinf(theta_it_ori_distance - theta_it_trans_distance_2),
                                                         cosf(theta_it_ori_distance - theta_it_trans_distance_2));
                float a_rotate_x_2 = pairs_a_b.a.x * cosf(diff_trans_to_ori_theta_2) -
                                     pairs_a_b.a.y * sinf(diff_trans_to_ori_theta) + car_x +
                                     car_y * sinf(diff_trans_to_ori_theta_2) - car_x * cosf(diff_trans_to_ori_theta);
                float a_rotate_y_2 = pairs_a_b.a.y * cosf(diff_trans_to_ori_theta_2) +
                                     pairs_a_b.a.x * sinf(diff_trans_to_ori_theta_2) + car_y -
                                     car_y * cosf(diff_trans_to_ori_theta_2) - car_x * sinf(diff_trans_to_ori_theta_2);
                float x_offset_2 = pairs_a_b.A.x - a_rotate_x_2;
                float y_offset_2 = pairs_a_b.A.y - a_rotate_y_2;
                std::vector<cv::Point2f>::iterator it_trans_to_2 = trans_points.begin();
                for (; it_trans_to_2 != trans_points.end(); ++it_trans_to_2)
                {
                    float tmp_trans_to_ori_x_2 = (*it_trans_to_2).x * cosf(diff_trans_to_ori_theta_2) -
                                                 (*it_trans_to_2).y * sinf(diff_trans_to_ori_theta_2) + car_x +
                                                 car_y * sinf(diff_trans_to_ori_theta_2) -
                                                 car_x * cosf(diff_trans_to_ori_theta_2);
                    float tmp_trans_to_ori_y_2 = (*it_trans_to_2).y * cosf(diff_trans_to_ori_theta_2) +
                                                 (*it_trans_to_2).x * sinf(diff_trans_to_ori_theta_2) + car_y -
                                                 car_y * cosf(diff_trans_to_ori_theta_2) -
                                                 car_x * sinf(diff_trans_to_ori_theta_2);

                    cv::Point2f it_trans_to_global_2;
                    it_trans_to_global_2.x = tmp_trans_to_ori_x_2 + x_offset_2;
                    it_trans_to_global_2.y = tmp_trans_to_ori_y_2 + y_offset_2;
                    std::vector<cv::Point2f>::iterator it_ori_2 = ori_points_2.begin();
                    for (; it_ori_2 != ori_points_2.end(); ++it_ori_2)
                    {
                        float dist = dist_between(it_trans_to_global_2, (*it_ori_2));
                        if (dist < 0.15)
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

    std::stable_sort(pairs_sample.begin(), pairs_sample.end(), sort_count);
    //    std::cout << (*pairs_sample.begin()).count << std::endl;
    match the_one = (*pairs_sample.begin());
    float pairs_trans_theta = atan2f(the_one.b.y - the_one.a.y, the_one.b.x - the_one.a.x);
    float pairs_oir_theta = atan2f(the_one.B.y - the_one.A.y, the_one.B.x - the_one.A.x);
    // a-b逆时钟旋转角度到地图A-B,车的姿态也要加上这个旋转角度（逆时针）
    float pairs_diffs = atan2f(sinf(pairs_oir_theta - pairs_trans_theta), cosf(pairs_oir_theta - pairs_trans_theta));
    //    std::cout << std::endl << "the trans theta is " << pairs_diffs / 3.1415 * 180 << std::endl;
    float x0 = the_one.a.x * cosf(pairs_diffs) - the_one.a.y * sinf(pairs_diffs) + car_x + car_y * sinf(pairs_diffs) -
               car_x * cosf(pairs_diffs);
    float y0 = the_one.a.y * cosf(pairs_diffs) + the_one.a.x * sinf(pairs_diffs) + car_y - car_y * cosf(pairs_diffs) -
               car_x * sinf(pairs_diffs);

    float car_offset_x = the_one.A.x - x0;
    float car_offset_y = the_one.A.y - y0;
    //    std::cout << "the car offset is " << car_offset_x << "," << car_offset_y << std::endl;

    rotated_pose[0] = estimate_pose[0] + car_offset_x;
    rotated_pose[1] = estimate_pose[1] + car_offset_y;
    rotated_pose[2] = atan2f(sinf(estimate_pose[2] + pairs_diffs), cosf(estimate_pose[2] + pairs_diffs));

    std::cout << "\033[1;31m Rotated Pose \033[0m " << the_one.count << std::endl;
    std::cout << "\033[1;31m Rotated Pose \033[0m " << rotated_pose << std::endl;

    return the_one.count;
}

//    class graphOptmizer {
//
//    public:
//        void init();
//
//        void addVertexToGraph(Eigen::Matrix4f &pose_to_add, double pose_time);
//
//        void addEdgeToGraph(Eigen::Matrix4f transform_between_vertexes,
//                            Id id1, Id id2);
//
////    void addLoopClosureToGraph(Eigen::Matrix4f transform_for_loop,
////                               Id id1, Id id2);
//
//        g2o::SparseOptimizer trajectory_optimizer_;
//
//        std::unordered_map <Id, Eigen::Vector3f> updateGraph();
//
//    private:
//        ///std::unordered_map<Id, Eigen::Vector3f> pose_sequence_;
//        int vertex_id = 1;
////        std::vector <Id> Id_list_;
////        std::unordered_map<Id, Eigen::Vector3f> pose_sequence_;
//    };

#endif //PROJECT_GRAPHSLAM_H
