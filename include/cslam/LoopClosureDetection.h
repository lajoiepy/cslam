#include "loop_closure_detection/LoopClosureServiceHandler.h"

/**
 * @brief Loop Closure Detection Management
 * - Receives keyframes from RTAB-map
 * - Generate keypoints from frames
 * - Sends/Receives keypoints from other robot frames
 * - Computes geometric verification *
 */
class LoopClosureDetection {
public:
  LoopClosureDetection(){};
  ~LoopClosureDetection(){};

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
   * @brief Receives the keyframe data from RTAB-map
   *
   * @param map_data_msg keyframe data
   * @param info_msg keyframe statistics
   */
  void mapDataCallback(
      const std::shared_ptr<rtabmap_ros::msg::MapData> &map_data_msg,
      const std::shared_ptr<rtabmap_ros::msg::Info> &info_msg);

  /**
   * @brief Service callback to compute local descriptors and publishing them.
   *
   * @param request Images IDs
   * @param response Success flag
   */
  void sendLocalImageDescriptors(
      const std::shared_ptr<
          cslam_loop_detection::srv::SendLocalImageDescriptors::Request>
          request,
      std::shared_ptr<
          cslam_loop_detection::srv::SendLocalImageDescriptors::Response>
          response);

  /**
   * @brief Message callback to receive keypoints and compute
   *
   * @param msg local descriptors
   */
  void receiveLocalImageDescriptors(
      const std::shared_ptr<cslam_loop_detection::msg::LocalImageDescriptors>
          msg);

private:
  LoopClosureServiceHandler loop_closure_detector_;

  rclcpp::Client<rtabmap_ros::srv::AddLink>::SharedPtr add_link_srv_;

  rclcpp::Service<cslam_loop_detection::srv::SendLocalImageDescriptors>::
      SharedPtr send_local_descriptors_srv_;

  std::map<int, rtabmap::SensorData> local_data_;

  std::shared_ptr<rclcpp::Node> node_;

  std::deque<std::shared_ptr<rtabmap_ros::msg::MapData>> received_data_queue_;

  int max_queue_size_, min_inliers_, nb_robots_, robot_id_;

  std::map<int,
           rclcpp::Publisher<
               cslam_loop_detection::msg::LocalImageDescriptors>::SharedPtr>
      local_descriptors_publishers_;

  rclcpp::Subscription<cslam_loop_detection::msg::LocalImageDescriptors>::
      SharedPtr local_descriptors_subscriber_;

  rtabmap::RegistrationVis registration_;
};