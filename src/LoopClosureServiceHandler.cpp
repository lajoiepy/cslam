#include "loop_closure_detection/LoopClosureServiceHandler.h"

void LoopClosureServiceHandler::init(std::shared_ptr<rclcpp::Node> &node) {
  node_ = node;
  client_ = node_->create_client<cslam_loop_detection::srv::DetectLoopClosure>(
      "detect_loop_closure");
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }
}

void LoopClosureServiceHandler::detectLoopClosures(
    const rtabmap::SensorData &data, const int id) {
  RCLCPP_INFO(node_->get_logger(),
              "Process Image %d for Loop Closure Detection", id);
  // Image message
  std_msgs::msg::Header header;
  header.stamp = node_->now();
  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::RGB8, data.imageRaw());
  cslam_utils::msg::ImageId image_msg;
  image_bridge.toImageMsg(image_msg.image);
  image_msg.id = id;

  // Service request
  auto request =
      std::make_shared<cslam_loop_detection::srv::DetectLoopClosure::Request>();
  request->image = image_msg;

  responses_.push_back(client_->async_send_request(request));
  RCLCPP_INFO(node_->get_logger(),
              "Service called for place recognition processing");
}

LoopClosureResponse LoopClosureServiceHandler::checkForResponse() {
  LoopClosureResponse res = {false, false, -1, -1};
  if (!responses_.empty()) {
    auto response = responses_.front();
    auto status = response.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready ||
        status == std::future_status::deferred) {
      responses_.pop_front();
      int test = response.get()->from_id;
      res = {true, response.get()->is_detected, response.get()->from_id,
             response.get()->detected_loop_closure_id};
      RCLCPP_INFO(node_->get_logger(), "Image %d processed.", res.from_id);
    }
  }
  return res;
}