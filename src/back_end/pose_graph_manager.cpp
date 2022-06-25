#include "cslam/back_end/pose_graph_manager.h"

PoseGraphManager::PoseGraphManager(std::shared_ptr<rclcpp::Node> &node): node_(node) {

  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);
  node_->get_parameter("pose_graph_manager_process_period_ms", pose_graph_manager_process_period_ms_);

  odometry_subscriber_ = node->create_subscription<
        nav_msgs::msg::Odometry>(
        "odom", 100,
        std::bind(&PoseGraphManager::odometry_callback, this,
                    std::placeholders::_1));
}

void PoseGraphManager::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    
}