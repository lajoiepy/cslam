#ifndef GTSAMMSGCONVERSION_H_
#define GTSAMMSGCONVERSION_H_

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cslam_common_interfaces/msg/pose_graph.hpp>
#include <cslam_common_interfaces/msg/pose_graph_edge.hpp>
#include <cslam_common_interfaces/msg/pose_graph_value.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define GRAPH_LABEL 'g'
#define ROBOT_LABEL(id) ('A' + id)
#define ROBOT_ID(ch) ((unsigned int)(ch - 'A'))

namespace cslam {

/**
 * @brief Converts odometry message to gtsam::Pose3
 *
 * @param odom_msg Odometry message
 * @return pose Pose data
 */
gtsam::Pose3 odometry_msg_to_pose3(const nav_msgs::msg::Odometry &odom_msg);

/**
 * @brief Converts a transform msg into a gtsam::Pose3
 *
 * @param msg Transform message
 * @return pose Pose data
 */
gtsam::Pose3 transform_msg_to_pose3(const geometry_msgs::msg::Transform &msg);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param pose 
 * @return geometry_msgs::msg::Pose 
 */
geometry_msgs::msg::Pose gtsam_pose_to_msg(const gtsam::Pose3 &pose);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param pose 
 * @return geometry_msgs::msg::Transform 
 */
geometry_msgs::msg::Transform
gtsam_pose_to_transform_msg(const gtsam::Pose3 &pose);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param values 
 * @return std::vector<cslam_common_interfaces::msg::PoseGraphValue> 
 */
std::vector<cslam_common_interfaces::msg::PoseGraphValue>
gtsam_values_to_msg(const gtsam::Values::shared_ptr values);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param factors 
 * @return std::vector<cslam_common_interfaces::msg::PoseGraphEdge> 
 */
std::vector<cslam_common_interfaces::msg::PoseGraphEdge>
gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph::shared_ptr factors);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param values 
 * @return std::vector<cslam_common_interfaces::msg::PoseGraphValue> 
 */
std::vector<cslam_common_interfaces::msg::PoseGraphValue>
gtsam_values_to_msg(const gtsam::Values &values);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param factors 
 * @return std::vector<cslam_common_interfaces::msg::PoseGraphEdge> 
 */
std::vector<cslam_common_interfaces::msg::PoseGraphEdge>
gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph &factors);

/**
 * @brief Converts from ROS 2 message to GTSAM
 * 
 * @param pose 
 * @return gtsam::Pose3 
 */
gtsam::Pose3 pose_msg_to_gtsam(const geometry_msgs::msg::Pose &pose);

/**
 * @brief Converts from ROS 2 message to GTSAM
 * 
 * @param values 
 * @return gtsam::Values::shared_ptr 
 */
gtsam::Values::shared_ptr values_msg_to_gtsam(
    const std::vector<cslam_common_interfaces::msg::PoseGraphValue> &values);

/**
 * @brief Converts from ROS 2 message to GTSAM
 * 
 * @param edges 
 * @return gtsam::NonlinearFactorGraph::shared_ptr 
 */
gtsam::NonlinearFactorGraph::shared_ptr edges_msg_to_gtsam(
    const std::vector<cslam_common_interfaces::msg::PoseGraphEdge> &edges);

} // namespace cslam

#endif