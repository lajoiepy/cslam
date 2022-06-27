#ifndef _POSEGRAPHMANAGER_H_
#define _POSEGRAPHMANAGER_H_

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
 
#include <nav_msgs/msg/odometry.hpp>

class PoseGraphManager {
public:
  /**
   * @brief Initialization of parameters and ROS 2 objects
   *
   * @param node ROS 2 node handle
   */
  PoseGraphManager(std::shared_ptr<rclcpp::Node> &node);
  ~PoseGraphManager(){};

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

private:

  // TODO: document
  std::shared_ptr<rclcpp::Node> node_;

  unsigned int nb_robots_, robot_id_;

  int pose_graph_manager_process_period_ms_;

  gtsam::NonlinearFactorGraph::shared_ptr graph_;
  gtsam::Values::shared_ptr estimate_;

  rclcpp::Subscription<
      nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscriber_;

};

#endif