#include "loop_closure_detection/LoopClosureDetectionSrvHandler.h"

class LoopClosureDetection {
public:
  LoopClosureDetection(){};
  ~LoopClosureDetection(){};

  void init(std::shared_ptr<rclcpp::Node> &node);

  void processNewKeyFrames();

  void geometricVerification();

  void mapDataCallback(
      const std::shared_ptr<rtabmap_ros::msg::map_data> &map_data_msg,
      const std::shared_ptr<rtabmap_ros::msg::Info> &info_msg);

  void sendLocalImageDescriptors(
      const std::shared_ptr<
          cslam_loop_detection::srv::SendLocalImageDescriptors::Request>
          request,
      std::shared_ptr<
          cslam_loop_detection::srv::SendLocalImageDescriptors::Response>
          response);

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

  std::deque<std::shared_ptr<rtabmap_ros::msg::map_data>> received_data_queue_;

  int max_queue_size_, min_inliers_, nb_robots_, robot_id_;

  std::map<int,
           rclcpp::Publisher<
               cslam_loop_detection::msg::LocalImageDescriptors>::SharedPtr>
      local_descriptors_publishers_;

  rclcpp::Subscription<cslam_loop_detection::msg::LocalImageDescriptors>::
      SharedPtr local_descriptors_subscriber_;

  rtabmap::RegistrationVis registration_;
};