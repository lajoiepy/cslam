#include "cslam/MapManager.h"
#include "cslam/StereoHandler.h"

template <class DataHandlerType>
MapManager<DataHandlerType>::MapManager(std::shared_ptr<rclcpp::Node> &node): node_(node),
  local_data_handler_(node) {

  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);

  RCLCPP_INFO(node_->get_logger(), "Initialization done.");
}

template <class DataHandlerType>
void MapManager<DataHandlerType>::process_new_keyframes() {
  local_data_handler_.process_new_keyframe();
}
// TODO: intra-robot loop closures
