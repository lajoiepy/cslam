#include "cslam/front_end/map_manager.h"

/**
 * @brief Node to interface with RTAB-map library for keyframe and 3D features
 * handling
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
  

  auto lcd = std::make_shared<MapManager<StereoHandler>>(node);

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    lcd->process_new_keyframes();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
