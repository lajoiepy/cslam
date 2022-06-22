#include "cslam/front_end/map_manager.h"

/**
 * @brief Node to manage the sensor data and registration
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("map_manager");

  node->declare_parameter<int>("pnp_min_inliers", 20);
  node->declare_parameter<int>("max_keyframe_queue_size", 10);
  node->declare_parameter<int>("nb_robots", 1);
  node->declare_parameter<int>("robot_id", 0);
  node->declare_parameter<int>("map_manager_process_period_ms", 100);

  auto lcd = std::make_shared<MapManager<StereoHandler>>(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
