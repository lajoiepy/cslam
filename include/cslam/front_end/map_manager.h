#ifndef _MAPMANAGER_H_
#define _MAPMANAGER_H_

#include <rclcpp/rclcpp.hpp>

#include <chrono>

#include "cslam/front_end/stereo_handler.h"

/**
 * @brief Loop Closure Detection Management
 * - Receives keyframes from RTAB-map
 * - Generate keypoints from frames
 * - Sends/Receives keypoints from other robot frames
 * - Computes geometric verification
 * 
 * @tparam DataHandlerType Depends on the type of input data (stereo, rgbd, etc.)
 */
template <class DataHandlerType>
class MapManager {
public:
  /**
   * @brief Initialization of parameters and ROS 2 objects
   *
   * @param node ROS 2 node handle
   */
  MapManager(std::shared_ptr<rclcpp::Node> &node);
  ~MapManager(){};

  /**
   * @brief Looks for loop closures in the current keyframe queue
   *
   */
  void process_new_keyframes();

private:

  // TODO: document
  std::shared_ptr<rclcpp::Node> node_;

  unsigned int nb_robots_, robot_id_;

  int map_manager_process_period_ms_;

  DataHandlerType local_data_handler_;

  rclcpp::TimerBase::SharedPtr process_timer_;
};

// List possible data types for C++ linker
template class MapManager<StereoHandler>;

#endif