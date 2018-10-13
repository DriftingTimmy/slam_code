//
// Created by lx on 18-6-15.
//

#include "graphSlam.h"

namespace LxSlam {

    void graphOptmizer::graphOptmizer() {

    }

    void graphOptmizer::init() {

        SlamLinearSolver* linearSolver = new SlamLinearSolver();
        linearSolver->setBlockOrdering( false );
        SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
        g2o::OptimizationAlgorithmLevenberg* solver =
                new g2o::OptimizationAlgorithmLevenberg( blockSolver );
        trajectory_optimizer_.setAlgorithm( solver );
        ///no debug information out
        trajectory_optimizer_.setVerbose( false );
        //optimize_step_ = save_num_ ;

        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId( 0 );
        v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
        v->setFixed( true ); //第一个顶点固定，不用优化
        trajectory_optimizer_.addVertex( v );
    }

    void graphOptmizer::addVertexToGraph(Eigen::Vector3f& pose_to_add,
                                         double pose_time) {
        Eigen::Matrix4f trans_pose = pose_to_matrix(pose_to_add[0], pose_to_add[1],
                                                    0, 0, 0, pose_to_add[2]);
        Eigen::Isometry3d vertex_transform;
        Matrix_to_Isometry(trans_pose,vertex_transform);

        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(vertex_id);
        v->setEstimate(vertex_transform);/// vertex pose = current pose
        trajectory_optimizer_.addVertex(v);

        Id pose_id;
        pose_id.counter = vertex_id;
        pose_id.time_ns = pose_time;
        pose_sequence_.insert({pose_id, pose_to_add});
        Id_list_.push_back(pose_id);
        vertex_id++;
    }

    void graphOptmizer::addEdgeToGraph(Eigen::Matrix4f transform_between_vertexes,
                                       Id id1, Id id2) {
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

        edge->setVertex(0, trajectory_optimizer_.vertex(id1.counter));
        edge->setVertex(1, trajectory_optimizer_.vertex(id2.counter));
        edge->setRobustKernel(new g2o::RobustKernelHuber());

        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
//
//    information(0, 0) = information(1, 1) = information(2, 2) = 100;
//    information(3, 3) = information(4, 4) = information(5, 5) = 100;
        ///信息矩阵的元素应该随置信度而变化，有待修改

        edge->setInformation(information);

        Eigen::Isometry3d T;
        Matrix_to_Isometry(transform_between_vertexes,T);

        edge->setMeasurement(T);

        trajectory_optimizer_.addEdge(edge);

    }

//void graphOptmizer::addLoopClosureToGraph(Eigen::Matrix4f transform_for_loop,
//                                          Id id1, Id id2) {
//
//}

    std::unordered_map<Id, Eigen::Vector3f> graphOptmizer::updateGraph() {

        trajectory_optimizer_.initializeOptimization();
        trajectory_optimizer_.optimize(60);

        for(auto i = 0; i < vertex_id; i++){
            g2o::VertexSE3* v =
                    dynamic_cast<g2o::VertexSE3*> (trajectory_optimizer_.vertex(i));
            Eigen::Isometry3d new_trans = v->estimate();
            Eigen::Matrix4f transform_updated;
            Isometry_to_Matrix(new_trans, transform_updated);
            Eigen::Vector3d updated_pose;
            double z, roll, pitch;
            matrix_to_pose(transform_updated, updated_pose[0],
                           updated_pose[1], z, roll, pitch,
                           updated_pose[2]);
            pose_sequence_.at(Id_list_.at(i)) = updated_pose;
        }
///Update the corresponding pose_sequence to update the target map;
        return pose_sequence_;
    }
}