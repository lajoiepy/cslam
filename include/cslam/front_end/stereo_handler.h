#ifndef _STEREOHANDLER_H_
#define _STEREOHANDLER_H_

#include <rclcpp/rclcpp.hpp>

#include <rtabmap_ros/msg/rgbd_image.hpp>
#include <rtabmap_ros/srv/add_link.hpp>
#include <rtabmap_ros/srv/get_map.hpp>

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

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cslam_common_interfaces/msg/keyframe_rgb.hpp>
#include <cslam_loop_detection_interfaces/msg/local_image_descriptors.hpp>
#include <cslam_loop_detection_interfaces/srv/send_local_image_descriptors.hpp>
#include <cslam_loop_detection_interfaces/msg/inter_robot_loop_closure.hpp>
#include <deque>
#include <functional>
#include <thread>

#include <memory>

#include "cslam/front_end/msg_conversion.h"

class StereoHandler {
public:

  /**
   * @brief Initialization of parameters and ROS 2 objects
   *
   * @param node ROS 2 node handle
   */
  StereoHandler(std::shared_ptr<rclcpp::Node> &node);
  ~StereoHandler(){};

  /**
   * @brief Processes Latest received image
   * 
   */
  void process_new_keyframe();

  /**
   * @brief Service callback to compute local descriptors and publishing them.
   *
   * @param request Images IDs
   * @param response Success flag
   */
  void send_local_image_descriptors(
      const std::shared_ptr<cslam_loop_detection_interfaces::srv::
                                SendLocalImageDescriptors::Request>
          request,
      std::shared_ptr<cslam_loop_detection_interfaces::srv::
                          SendLocalImageDescriptors::Response>
          response);

  /**
   * @brief Message callback to receive descriptors and compute
   *
   * @param msg local descriptors
   */
  void receive_local_image_descriptors(
      const std::shared_ptr<
          cslam_loop_detection_interfaces::msg::LocalImageDescriptors>
          msg);

  /**
   * @brief Computes local 3D descriptors from frame data and store them
   * 
   * @param frame_data Full frame data
   */
  void compute_local_descriptors(std::shared_ptr<rtabmap::SensorData>& frame_data);

  /**
   * @brief converts descriptors to sensore data
   * 
   * @param msg local descriptors
   * @return rtabmap::SensorData& 
   */
  void local_descriptors_msg_to_sensor_data(const std::shared_ptr<
        cslam_loop_detection_interfaces::msg::LocalImageDescriptors>
        msg, rtabmap::SensorData& sensor_data);

  /**
   * @brief converts sensor data to descriptor msg
   * 
   * @param sensor_data local descriptors
   * @param msg_data rtabmap_ros::msg::RGBDImage& 
   */
  void sensor_data_to_rgbd_msg(const std::shared_ptr<
        rtabmap::SensorData>
        sensor_data, rtabmap_ros::msg::RGBDImage& msg_data);

  /**
   * @brief Function to send the image to the python node
   *
   * @param data keyframe data
   * @param id keyframe id
   */
  void send_keyframe(const rtabmap::SensorData &data, const int id);

  /**
   * @brief Call back receiving sync data from camera
   * 
   * @param imageRectLeft 
   * @param imageRectRight 
   * @param cameraInfoLeft 
   * @param cameraInfoRight 
   */
  void stereo_callback(
			const sensor_msgs::msg::Image::ConstSharedPtr imageRectLeft,
			const sensor_msgs::msg::Image::ConstSharedPtr imageRectRight,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight);

private:

  std::deque<std::shared_ptr<rtabmap::SensorData>> received_data_queue_;
  std::map<int, std::shared_ptr<rtabmap::SensorData>> local_descriptors_map_;

  std::shared_ptr<rclcpp::Node> node_;

  unsigned int min_inliers_, nb_robots_, robot_id_, max_queue_size_, nb_local_frames_;

  image_transport::SubscriberFilter imageRectLeft_;
  image_transport::SubscriberFilter imageRectRight_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoLeft_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoRight_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> ExactSyncPolicy;
  message_filters::Synchronizer<ExactSyncPolicy> * exactSync_;

  rclcpp::Service<
      cslam_loop_detection_interfaces::srv::SendLocalImageDescriptors>::
      SharedPtr send_local_descriptors_srv_;

  std::map<int, rclcpp::Publisher<cslam_loop_detection_interfaces::msg::
                                      LocalImageDescriptors>::SharedPtr>
      local_descriptors_publishers_;

  rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeRGB>::SharedPtr
      keyframe_data_publisher_;

  rclcpp::Subscription<
      cslam_loop_detection_interfaces::msg::LocalImageDescriptors>::SharedPtr
      local_descriptors_subscriber_;

  rtabmap::RegistrationVis registration_;

  std::map<int, rclcpp::Publisher<cslam_loop_detection_interfaces::msg::
                                      InterRobotLoopClosure>::SharedPtr>
      inter_robot_loop_closure_publishers_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_frame_id_;
};
#endif