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
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define GRAPH_LABEL 'g'
#define ROBOT_LABEL(id) ('A'+id)
#define ROBOT_ID(ch) ((unsigned int)(ch - 'A'))

namespace cslam {

  /**
   * @brief Converts odometry message to gtsam::Pose3
   * 
   * @param odom_msg Odometry message
   * @return pose Pose data
   */
  gtsam::Pose3 odometry_msg_to_pose3(const nav_msgs::msg::Odometry& odom_msg);

  /**
   * @brief Converts a transform msg into a gtsam::Pose3
   * 
   * @param msg Transform message
   * @return pose Pose data
   */
  gtsam::Pose3 transform_msg_to_pose3(const geometry_msgs::msg::Transform& msg);

// TODO: document
geometry_msgs::msg::Pose gtsam_pose_to_msg(const gtsam::Pose3& pose);

geometry_msgs::msg::Transform gtsam_pose_to_transform_msg(const gtsam::Pose3& pose);

std::vector<cslam_common_interfaces::msg::PoseGraphValue> gtsam_values_to_msg(const gtsam::Values::shared_ptr values);

std::vector<cslam_common_interfaces::msg::PoseGraphEdge> gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph::shared_ptr factors);

std::vector<cslam_common_interfaces::msg::PoseGraphValue> gtsam_values_to_msg(const gtsam::Values& values);

std::vector<cslam_common_interfaces::msg::PoseGraphEdge> gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph& factors);

gtsam::Pose3 pose_msg_to_gtsam(const geometry_msgs::msg::Pose& pose);

gtsam::Values::shared_ptr values_msg_to_gtsam(const std::vector<cslam_common_interfaces::msg::PoseGraphValue>& values);

gtsam::NonlinearFactorGraph::shared_ptr edges_msg_to_gtsam(const std::vector<cslam_common_interfaces::msg::PoseGraphEdge>& edges);

}

#endif