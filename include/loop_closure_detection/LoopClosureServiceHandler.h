#include <rclcpp/rclcpp.hpp>

#include <rtabmap_ros/msg/info.hpp>
#include <rtabmap_ros/msg/map_data.hpp>
#include <rtabmap_ros/srv/add_link.hpp>
#include <rtabmap_ros/srv/get_map.hpp>

#include <rtabmap_ros/MsgConversion.h>

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UStl.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cslam_loop_detection/msg/local_image_descriptors.hpp>
#include <cslam_loop_detection/srv/detect_loop_closure.hpp>
#include <cslam_loop_detection/srv/send_local_image_descriptors.hpp>
#include <cslam_utils/msg/image_id.hpp>
#include <deque>
#include <functional>
#include <thread>

/**
 * @brief Response struct indicating if the loop closure is valid or detected
 * along with the corresponding IDs.
 *
 */
struct LoopClosureResponse {
  bool is_valid;
  bool is_detected;
  int from_id;
  int to_id;
};

/**
 * @brief Interface with python script for loop closure detection with global
 * image matching (e.g., NetVLAD)
 *
 */
class LoopClosureServiceHandler {
public:
  LoopClosureServiceHandler(){};

  ~LoopClosureServiceHandler(){};

  /**
   * @brief Initialization of service client
   *
   * @param node ROS 2 node handle
   */
  void init(std::shared_ptr<rclcpp::Node> &node);

  /**
   * @brief Asynchronous call for loop closure detection python service
   *
   * @param data keyframe data
   * @param id keyframe id
   */
  void detectLoopClosures(const rtabmap::SensorData &data, const int id);

  /**
   * @brief Look for response of asynchronous call
   *
   * @return LoopClosureResponse
   */
  LoopClosureResponse checkForResponse();

private:
  rclcpp::Client<cslam_loop_detection::srv::DetectLoopClosure>::SharedPtr
      client_;
  std::deque<rclcpp::Client<
      cslam_loop_detection::srv::DetectLoopClosure>::SharedFuture>
      responses_;
  std::shared_ptr<rclcpp::Node> node_;
};