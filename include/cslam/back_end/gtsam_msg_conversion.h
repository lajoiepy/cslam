#ifndef GTSAMMSGCONVERSION_H_
#define GTSAMMSGCONVERSION_H_

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/linear/NoiseModel.h>
 
#include <cslam_common_interfaces/msg/pose_graph.hpp>
#include <cslam_common_interfaces/msg/pose_graph_edge.hpp>
#include <cslam_common_interfaces/msg/pose_graph_value.hpp>

namespace cslam {

geometry_msgs::msg::Pose gtsam_pose_to_msg(const gtsam::Pose3& pose);

std::vector<cslam_common_interfaces::msg::PoseGraphValue> gtsam_values_to_msg(const gtsam::Values::shared_ptr values);

std::vector<cslam_common_interfaces::msg::PoseGraphEdge> gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph::shared_ptr factors);

}

#endif