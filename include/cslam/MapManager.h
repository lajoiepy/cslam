#include <rclcpp/rclcpp.hpp>

#include <rtabmap_ros/msg/rgbd_image.hpp>
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
#include <cslam_common_interfaces/msg/keyframe_rgb.hpp>
#include <cslam_loop_detection_interfaces/msg/inter_robot_loop_closure.hpp>
#include <cslam_loop_detection_interfaces/msg/local_image_descriptors.hpp>
#include <cslam_loop_detection_interfaces/srv/send_local_image_descriptors.hpp>
#include <deque>
#include <functional>
#include <thread>

/**
 * @brief Loop Closure Detection Management
 * - Receives keyframes from RTAB-map
 * - Generate keypoints from frames
 * - Sends/Receives keypoints from other robot frames
 * - Computes geometric verification
 */
class MapManager {
public:
  MapManager(){};
  ~MapManager(){};

  /**
   * @brief Initialization of parameters and ROS 2 objects
   *
   * @param node ROS 2 node handle
   */
  void init(std::shared_ptr<rclcpp::Node> &node);

  /**
   * @brief Looks for loop closures in the current keyframe queue
   *
   */
  void processNewKeyFrames();

  /**
   * @brief Computes 3D relative pose transform from keypoints
   *
   */
  void geometricVerification();

  /**
   * @brief Receives the image data from odometry
   *
   * @param image_msg image data
   */
  void receiveRGBD(
      const std::shared_ptr<rtabmap_ros::msg::RGBDImage> image_msg);

  /**
   * @brief Service callback to compute local descriptors and publishing them.
   *
   * @param request Images IDs
   * @param response Success flag
   */
  void sendLocalImageDescriptors(
      const std::shared_ptr<cslam_loop_detection_interfaces::srv::
                                SendLocalImageDescriptors::Request>
          request,
      std::shared_ptr<cslam_loop_detection_interfaces::srv::
                          SendLocalImageDescriptors::Response>
          response);

  /**
   * @brief Message callback to receive keypoints and compute
   *
   * @param msg local descriptors
   */
  void receiveLocalImageDescriptors(
      const std::shared_ptr<
          cslam_loop_detection_interfaces::msg::LocalImageDescriptors>
          msg);

  /**
   * @brief Function to send the image to the python node
   *
   * @param data keyframe data
   * @param id keyframe id
   */
  void sendKeyframe(const rtabmap::SensorData &data, const int id);

private:

  rclcpp::Service<
      cslam_loop_detection_interfaces::srv::SendLocalImageDescriptors>::
      SharedPtr send_local_descriptors_srv_;

  std::map<int, rtabmap::SensorData> local_data_;

  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Subscription<
      rtabmap_ros::msg::RGBDImage>::SharedPtr
      rgbd_subscriber_;

  std::deque<std::shared_ptr<rtabmap_ros::msg::RGBDImage>> received_data_queue_;

  unsigned int max_queue_size_, min_inliers_, nb_robots_, robot_id_, nb_local_frames_;

  std::map<int, rclcpp::Publisher<cslam_loop_detection_interfaces::msg::
                                      LocalImageDescriptors>::SharedPtr>
      local_descriptors_publishers_;

  rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeRGB>::SharedPtr
      keyframe_data_publisher_;

  std::map<int, rclcpp::Publisher<cslam_loop_detection_interfaces::msg::
                                      InterRobotLoopClosure>::SharedPtr>
      inter_robot_loop_closure_publishers_;

  rclcpp::Subscription<
      cslam_loop_detection_interfaces::msg::LocalImageDescriptors>::SharedPtr
      local_descriptors_subscriber_;

  rtabmap::RegistrationVis registration_;
};