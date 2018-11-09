//
// Created by lx on 18-6-15.
//
//
//#include "graphSlam.h"
//
//
//
////void graphOptmizer::addLoopClosureToGraph(Eigen::Matrix4f transform_for_loop,
////                                          Id id1, Id id2) {
////
////}
//
////    std::unordered_map<Id, Eigen::Vector3f> graphOptmizer::updateGraph() {
////
////        trajectory_optimizer_.initializeOptimization();
////        trajectory_optimizer_.optimize(60);
////
////        for(auto i = 0; i < vertex_id; i++){
////            g2o::VertexSE3* v =
////                    dynamic_cast<g2o::VertexSE3*> (trajectory_optimizer_.vertex(i));
////            Eigen::Isometry3d new_trans = v->estimate();
////            Eigen::Matrix4f transform_updated;
////            Isometry_to_Matrix(new_trans, transform_updated);
////            Eigen::Vector3d updated_pose;
////            double z, roll, pitch;
////            matrix_to_pose(transform_updated, updated_pose[0],
////                           updated_pose[1], z, roll, pitch,
////                           updated_pose[2]);
////            pose_sequence_.at(Id_list_.at(i)) = updated_pose;
////        }
///////Update the corresponding pose_sequence to update the target map;
////        return pose_sequence_;
////    }