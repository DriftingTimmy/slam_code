//
// Created by lx on 18-4-25.
//

#ifndef PROJECT_GRAPHOPTMIZER_H
#define PROJECT_GRAPHOPTMIZER_H
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace LxSlam {
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverEigen <SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    struct Id {

        int counter;
        double time_ns;

    };

    class graphOptmizer {

    public:
        graphOptmizer();

        ~graphOptmizer();

        void init();

        void addVertexToGraph(Eigen::Vector3f &pose_to_add, double pose_time);

        void addEdgeToGraph(Eigen::Matrix4f transform_between_vertexes,
                            Id id1, Id id2);

//    void addLoopClosureToGraph(Eigen::Matrix4f transform_for_loop,
//                               Id id1, Id id2);

        std::unordered_map <Id, Eigen::Vector3f> updateGraph();

    private:
        g2o::SparseOptimizer trajectory_optimizer_;
        ///std::unordered_map<Id, Eigen::Vector3f> pose_sequence_;
        int vertex_id = 1;
        std::vector <Id> Id_list_;

    };
}///namespace LxSlam

#endif //PROJECT_GRAPHOPTMIZER_H
