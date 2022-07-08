#include "cslam/back_end/pose_graph_manager.h"

using namespace cslam;

/**
 * @brief Node to manage the pose graph data
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pose_graph_manager");

  node->declare_parameter<int>("nb_robots", 1);
  node->declare_parameter<int>("robot_id", 0);
  node->declare_parameter<int>("pose_graph_manager_process_period_ms", 1000);
  node->declare_parameter<int>("pose_graph_optimization_loop_period_ms", 100);
  node->declare_parameter<int>("max_waiting_time_sec", 100);
  node->declare_parameter<int>("heartbeat_period_sec", 1.0);

  PoseGraphManager manager(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
