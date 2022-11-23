#include "cslam/back_end/decentralized_pgo.h"

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

  node->declare_parameter<int>("max_nb_robots", 1);
  node->declare_parameter<int>("robot_id", 0);
  node->declare_parameter<int>("backend.pose_graph_optimization_start_period_ms", 1000);
  node->declare_parameter<int>("backend.pose_graph_optimization_loop_period_ms", 100);
  node->declare_parameter<int>("backend.max_waiting_time_sec", 100);
  node->declare_parameter<bool>("backend.enable_broadcast_tf_frames", true);
  node->declare_parameter<double>("neighbor_management.heartbeat_period_sec", 1.0);
  node->declare_parameter<bool>("evaluation.enable_logs", false);
  node->declare_parameter<std::string>("evaluation.log_folder", "");
  node->declare_parameter<bool>("evaluation.enable_gps_recording", false);
  node->declare_parameter<bool>("evaluation.enable_simulated_rendezvous", false);
  node->declare_parameter<std::string>("evaluation.rendezvous_schedule_file", "");
  node->declare_parameter<bool>("evaluation.enable_pose_timestamps_recording", false);
  node->declare_parameter<bool>("visualization.enable", false);
  node->declare_parameter<int>("visualization.publishing_period_ms", 0);

  DecentralizedPGO manager(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
