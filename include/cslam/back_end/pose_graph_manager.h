#ifndef _POSEGRAPHMANAGER_H_
#define _POSEGRAPHMANAGER_H_

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
 
#include <cslam_common_interfaces/msg/keyframe_odom.hpp>
#include <cslam_common_interfaces/msg/optimization_result.hpp>
#include <cslam_common_interfaces/msg/robot_ids.hpp>
#include <cslam_common_interfaces/msg/optimizer_state.hpp>
#include <cslam_loop_detection_interfaces/msg/inter_robot_loop_closure.hpp>

#include <std_msgs/msg/string.hpp>
#include <list>

#include "cslam/back_end/gtsam_msg_conversion.h"

namespace cslam {

enum OptimizerState { IDLE, WAITING, POSEGRAPH_COLLECTION, OPTIMIZATION };
  
class PoseGraphManager {
public:
  /**
   * @brief Initialization of parameters and ROS 2 objects
   *
   * @param node ROS 2 node handle
   */
  PoseGraphManager(std::shared_ptr<rclcpp::Node> &node);
  ~PoseGraphManager(){};

  /**
   * @brief Receives odometry msg + keyframe id
   * 
   * @param msg 
   */
  void odometry_callback(const cslam_common_interfaces::msg::KeyframeOdom::ConstSharedPtr msg);

  /**
   * @brief Receives inter-robot loop closures
   * 
   * @param msg 
   */
  void inter_robot_loop_closure_callback(const cslam_loop_detection_interfaces::msg::
  InterRobotLoopClosure::ConstSharedPtr msg);

  /**
   * @brief Receives current neighbors
   * 
   * @param msg 
   */
  void current_neighbors_callback(const cslam_common_interfaces::msg::
                                      RobotIds::ConstSharedPtr msg);

  /**
   * @brief Receives request for pose graph
   * 
   * @param msg 
   */
  void get_pose_graph_callback(const cslam_common_interfaces::msg::
                                      RobotIds::ConstSharedPtr msg);

  /**
   * @brief Receives pose graph
   * 
   * @param msg 
   */
  void pose_graph_callback(const cslam_common_interfaces::msg::
                                      PoseGraph::ConstSharedPtr msg);

  /**
   * @brief Starts pose graph optimization process every X ms (defined in config)
   * 
   */
  void optimization_callback();

  /**
   * @brief Step execution of the optimization State Machine
   * 
   */
  void optimization_loop_callback();

  /**
   * @brief Performs pose graph optimization
   * 
   */
  void perform_optimization();

  /**
   * @brief Request to check the current neighbors
   * 
   */
  void resquest_current_neighbors();

  /**
   * @brief Sets received pose graphs to false
   * 
   */
  void reinitialize_received_pose_graphs();

  /**
   * @brief Check if received all pose graphs
   * 
   */
  bool check_received_pose_graphs();

  /**
   * @brief Start waiting
   * 
   */
  void start_waiting();

  /**
   * @brief Check waiting timeout
   * 
   */
  bool check_waiting_timeout();

  /**
   * @brief End waiting
   * 
   */
  void end_waiting();

  /**
   * @brief Breadth First Seach to check connectivity
   * 
   */
  std::map<unsigned int, bool> connected_robot_pose_graph();

  /**
   * @brief Put all connected pose graphs into one
   * 
   */
  std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr> aggregate_pose_graphs();

private:

  // TODO: document
  std::shared_ptr<rclcpp::Node> node_;

  unsigned int nb_robots_, robot_id_;

  unsigned int pose_graph_manager_process_period_ms_, pose_graph_optimization_loop_period_ms_;

  gtsam::SharedNoiseModel default_noise_model_;
  float rotation_default_noise_std_, translation_default_noise_std_;

  gtsam::NonlinearFactorGraph::shared_ptr pose_graph_;
  gtsam::Values::shared_ptr current_pose_estimates_;

  std::map<unsigned int, std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>> other_robots_graph_and_estimates_;

  gtsam::Pose3 latest_local_pose_;
  gtsam::LabeledSymbol latest_local_symbol_;

  std::map<std::pair<unsigned int, unsigned int>, std::vector<gtsam::BetweenFactor<gtsam::Pose3>>> inter_robot_loop_closures_;

  rclcpp::Subscription<
      cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr
      odometry_subscriber_;

  rclcpp::Subscription<cslam_loop_detection_interfaces::msg::
                                      InterRobotLoopClosure>::SharedPtr
      inter_robot_loop_closure_subscriber_;

  rclcpp::Publisher<cslam_common_interfaces::msg::OptimizationResult>::SharedPtr
      optimization_result_publisher_;

  rclcpp::TimerBase::SharedPtr optimization_timer_, optimization_loop_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_current_neighbors_publisher_;

  rclcpp::Subscription<cslam_common_interfaces::msg::RobotIds>::SharedPtr current_neighbors_subscriber_;

  std::map<unsigned int, rclcpp::Publisher<cslam_common_interfaces::msg::RobotIds>::SharedPtr> get_pose_graph_publishers_;

  std::map<unsigned int, bool> received_pose_graphs_;

  std::map<unsigned int, std::vector<unsigned int>> received_pose_graphs_connectivity_;

  rclcpp::Subscription<cslam_common_interfaces::msg::RobotIds>::SharedPtr get_pose_graph_subscriber_;

  rclcpp::Subscription<cslam_common_interfaces::msg::PoseGraph>::SharedPtr pose_graph_subscriber_;

  rclcpp::Publisher<cslam_common_interfaces::msg::PoseGraph>::SharedPtr pose_graph_publisher_;

  cslam_common_interfaces::msg::RobotIds current_neighbors_ids_;

  OptimizerState optimizer_state_;

  rclcpp::Publisher<cslam_common_interfaces::msg::OptimizerState>::SharedPtr optimizer_state_publisher_;

  bool is_waiting_;

  rclcpp::Time start_waiting_time_;
  rclcpp::Duration max_waiting_time_sec_;
};
}
#endif