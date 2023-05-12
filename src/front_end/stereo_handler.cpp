#include "cslam/front_end/stereo_handler.h"
#include <rtabmap_conversions/MsgConversion.h>

using namespace rtabmap;
using namespace cslam;

StereoHandler::StereoHandler(std::shared_ptr<rclcpp::Node> &node)
    : RGBDHandler(node) {
  node->declare_parameter<std::string>("frontend.left_image_topic", "left/image_rect");
  node->declare_parameter<std::string>("frontend.right_image_topic", "right/image_rect");
  node->declare_parameter<std::string>("frontend.left_camera_info_topic",
                                       "left/camera_info");
  node->declare_parameter<std::string>("frontend.right_camera_info_topic",
                                       "right/camera_info");

  // Subscriber for stereo images
  sub_image_rect_left_.subscribe(
      node_.get(), node_->get_parameter("frontend.left_image_topic").as_string(), "raw",
      rclcpp::QoS(max_queue_size_)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  sub_image_rect_right_.subscribe(
      node_.get(), node_->get_parameter("frontend.right_image_topic").as_string(), "raw",
      rclcpp::QoS(max_queue_size_)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  sub_camera_info_left_.subscribe(
      node_.get(), node_->get_parameter("frontend.left_camera_info_topic").as_string(),
      rclcpp::QoS(max_queue_size_)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  sub_camera_info_right_.subscribe(
      node_.get(), node_->get_parameter("frontend.right_camera_info_topic").as_string(),
      rclcpp::QoS(max_queue_size_)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());

  stereo_sync_policy_ = new message_filters::Synchronizer<StereoSyncPolicy>(
      StereoSyncPolicy(max_queue_size_), sub_image_rect_left_, sub_image_rect_right_,
      sub_camera_info_left_, sub_camera_info_right_, sub_odometry_);
  stereo_sync_policy_->registerCallback(
      std::bind(&StereoHandler::stereo_callback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

}

void StereoHandler::stereo_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr image_rect_left,
    const sensor_msgs::msg::Image::ConstSharedPtr image_rect_right,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_left,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_right,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr) {
  // If odom tracking failed, do not process the frame
  if (odom_ptr->pose.covariance[0] > 1000)
  {
    RCLCPP_WARN(node_->get_logger(), "Odom tracking failed, skipping frame");
    return;
  }
  // Fix timestamps for logging
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*odom_ptr);
  odom->header.stamp = image_rect_left->header.stamp;
  
  if (!(image_rect_left->encoding.compare(
            sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
        image_rect_left->encoding.compare(sensor_msgs::image_encodings::MONO8) ==
            0 ||
        image_rect_left->encoding.compare(sensor_msgs::image_encodings::MONO16) ==
            0 ||
        image_rect_left->encoding.compare(sensor_msgs::image_encodings::BGR8) ==
            0 ||
        image_rect_left->encoding.compare(sensor_msgs::image_encodings::RGB8) ==
            0 ||
        image_rect_left->encoding.compare(sensor_msgs::image_encodings::BGRA8) ==
            0 ||
        image_rect_left->encoding.compare(sensor_msgs::image_encodings::RGBA8) ==
            0) ||
      !(image_rect_right->encoding.compare(
            sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
        image_rect_right->encoding.compare(sensor_msgs::image_encodings::MONO8) ==
            0 ||
        image_rect_right->encoding.compare(
            sensor_msgs::image_encodings::MONO16) == 0 ||
        image_rect_right->encoding.compare(sensor_msgs::image_encodings::BGR8) ==
            0 ||
        image_rect_right->encoding.compare(sensor_msgs::image_encodings::RGB8) ==
            0 ||
        image_rect_right->encoding.compare(sensor_msgs::image_encodings::BGRA8) ==
            0 ||
        image_rect_right->encoding.compare(sensor_msgs::image_encodings::RGBA8) ==
            0)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (mono8 "
        "recommended), received types are %s (left) and %s (right)",
        image_rect_left->encoding.c_str(), image_rect_right->encoding.c_str());
    return;
  }

  rclcpp::Time stamp =
      rtabmap_conversions::timestampFromROS(image_rect_left->header.stamp) >
              rtabmap_conversions::timestampFromROS(image_rect_right->header.stamp)
          ? image_rect_left->header.stamp
          : image_rect_right->header.stamp;

  Transform localTransform(0,0,0,0,0,0);
  if (base_frame_id_ != "")
  {
		localTransform = rtabmap_conversions::getTransform(
		    base_frame_id_, image_rect_left->header.frame_id, stamp, *tf_buffer_, 0.1);
		if (localTransform.isNull()) {
		  RCLCPP_INFO(node_->get_logger(),
		               "Could not get transform from %s to %s after 0.1 s!",
		               base_frame_id_.c_str(), image_rect_left->header.frame_id.c_str());
		  return;
		}
	}

  if (image_rect_left->data.size() && image_rect_right->data.size()) {
    bool alreadyRectified = true;
    rtabmap::Transform stereoTransform;
    if (!alreadyRectified) {
      stereoTransform = rtabmap_conversions::getTransform(
          camera_info_right->header.frame_id, camera_info_left->header.frame_id,
          camera_info_left->header.stamp, *tf_buffer_, 0.1);
      if (stereoTransform.isNull()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter %s is false but we cannot get TF between the "
                     "two cameras! (between frames %s and %s)",
                     Parameters::kRtabmapImagesAlreadyRectified().c_str(),
                     camera_info_right->header.frame_id.c_str(),
                     camera_info_left->header.frame_id.c_str());
        return;
      } else if (stereoTransform.isIdentity()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter %s is false but we cannot get a valid TF "
                     "between the two cameras! "
                     "Identity transform returned between left and right "
                     "cameras. Verify that if TF between "
                     "the cameras is valid: \"rosrun tf tf_echo %s %s\".",
                     Parameters::kRtabmapImagesAlreadyRectified().c_str(),
                     camera_info_right->header.frame_id.c_str(),
                     camera_info_left->header.frame_id.c_str());
        return;
      }
    }

    rtabmap::StereoCameraModel stereoModel =
        rtabmap_conversions::stereoCameraModelFromROS(*camera_info_left, *camera_info_right,
                                              localTransform, stereoTransform);

    if (stereoModel.baseline() == 0 && alreadyRectified) {
      stereoTransform = rtabmap_conversions::getTransform(
          camera_info_left->header.frame_id, camera_info_right->header.frame_id,
          camera_info_left->header.stamp, *tf_buffer_, 0.1);

      if (!stereoTransform.isNull() && stereoTransform.x() > 0) {
        static bool warned = false;
        if (!warned) {
          RCLCPP_WARN(
              node_->get_logger(),
              "Right camera info doesn't have Tx set but we are assuming that "
              "stereo images are already rectified (see %s parameter). While "
              "not "
              "recommended, we used TF to get the baseline (%s->%s = %fm) for "
              "convenience (e.g., D400 ir stereo issue). It is preferred to "
              "feed "
              "a valid right camera info if stereo images are already "
              "rectified. This message is only printed once...",
              rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
              camera_info_right->header.frame_id.c_str(),
              camera_info_left->header.frame_id.c_str(), stereoTransform.x());
          warned = true;
        }
        stereoModel = rtabmap::StereoCameraModel(
            stereoModel.left().fx(), stereoModel.left().fy(),
            stereoModel.left().cx(), stereoModel.left().cy(),
            stereoTransform.x(), stereoModel.localTransform(),
            stereoModel.left().imageSize());
      }
    }

    if (alreadyRectified && stereoModel.baseline() <= 0) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "The stereo baseline (%f) should be positive (baseline=-Tx/fx). We "
          "assume a horizontal left/right stereo "
          "setup where the Tx (or P(0,3)) is negative in the right camera info "
          "msg.",
          stereoModel.baseline());
      return;
    }

    if (stereoModel.baseline() > 10.0) {
      static bool shown = false;
      if (!shown) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Detected baseline (%f m) is quite large! Is your "
            "right camera_info P(0,3) correctly set? Note that "
            "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
            stereoModel.baseline());
        shown = true;
      }
    }

    cv_bridge::CvImagePtr ptrImageLeft = cv_bridge::toCvCopy(
        image_rect_left, image_rect_left->encoding.compare(
                           sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
                               image_rect_left->encoding.compare(
                                   sensor_msgs::image_encodings::MONO8) == 0
                           ? ""
                       : image_rect_left->encoding.compare(
                             sensor_msgs::image_encodings::MONO16) != 0
                           ? "bgr8"
                           : "mono8");
    cv_bridge::CvImagePtr ptrImageRight = cv_bridge::toCvCopy(
        image_rect_right, image_rect_right->encoding.compare(
                            sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
                                image_rect_right->encoding.compare(
                                    sensor_msgs::image_encodings::MONO8) == 0
                            ? ""
                            : "mono8");

    auto data = std::make_shared<rtabmap::SensorData>(
        ptrImageLeft->image, ptrImageRight->image, stereoModel,
        0, rtabmap_conversions::timestampFromROS(stamp));

    received_data_queue_.push_back(std::make_pair(data, odom));
    if (received_data_queue_.size() > max_queue_size_) {
      // Remove the oldest keyframes if we exceed the maximum size
      received_data_queue_.pop_front();
      RCLCPP_WARN(
          node_->get_logger(),
          "Stereo: Maximum queue size (%d) exceeded, the oldest element was removed.",
          max_queue_size_);
    }

    if (enable_gps_recording_) {
        received_gps_queue_.push_back(latest_gps_fix_);
        if (received_gps_queue_.size() > max_queue_size_)
        {
        received_gps_queue_.pop_front();
        }
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Odom: input images empty?!");
  }
}

void StereoHandler::local_descriptors_msg_to_sensor_data(
    const std::shared_ptr<
        cslam_common_interfaces::msg::LocalImageDescriptors>
        msg,
    rtabmap::SensorData &sensor_data) {
  // Fill descriptors
  rtabmap::StereoCameraModel stereo_model =
      rtabmap_conversions::stereoCameraModelFromROS(msg->data.rgb_camera_info,
                                            msg->data.depth_camera_info,
                                            rtabmap::Transform::getIdentity());
  sensor_data = rtabmap::SensorData(
      cv::Mat(), cv::Mat(), stereo_model, 0,
      rtabmap_conversions::timestampFromROS(msg->data.header.stamp));

  std::vector<cv::KeyPoint> kpts;
  rtabmap_conversions::keypointsFromROS(msg->data.key_points, kpts);
  std::vector<cv::Point3f> kpts3D;
  rtabmap_conversions::points3fFromROS(msg->data.points, kpts3D);
  auto descriptors = rtabmap::uncompressData(msg->data.descriptors);
  sensor_data.setFeatures(kpts, kpts3D, descriptors);
}

void StereoHandler::send_visualization(const std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<const nav_msgs::msg::Odometry>> &keypoints_data)
{
  send_visualization_keypoints(keypoints_data);
}
