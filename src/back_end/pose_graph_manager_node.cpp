#include "cslam/back_end/pose_graph_manager.h"

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

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
