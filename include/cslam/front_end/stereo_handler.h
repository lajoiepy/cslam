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
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cslam_common_interfaces/msg/keyframe_rgb.hpp>
#include <cslam_common_interfaces/msg/keyframe_odom.hpp>
#include <cslam_loop_detection_interfaces/msg/local_image_descriptors.hpp>
#include <cslam_loop_detection_interfaces/msg/inter_robot_loop_closure.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cslam_loop_detection_interfaces/msg/local_descriptors_request.hpp>
#include <deque>
#include <functional>
#include <thread>

#include <memory>

#include "cslam/front_end/sensor_msg_utils.h"

namespace cslam {

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
   * @brief Service callback to publish local descriptors
   *
   * @param request Image ID to send and matching info
   */
  void local_descriptors_request(
     cslam_loop_detection_interfaces::msg::LocalDescriptorsRequest::ConstSharedPtr
          request);

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
   * TODO: Move to parent class
   *
   * @param data keyframe data
   * @param id keyframe id
   */
  void send_keyframe(const rtabmap::SensorData &data,  const nav_msgs::msg::Odometry::ConstSharedPtr odom, const int id);

  /**
   * @brief Callback receiving sync data from camera
   * 
   * @param imageRectLeft 
   * @param imageRectRight 
   * @param cameraInfoLeft 
   * @param cameraInfoRight 
   * @param odom 
   */
  void stereo_callback(
			const sensor_msgs::msg::Image::ConstSharedPtr imageRectLeft,
			const sensor_msgs::msg::Image::ConstSharedPtr imageRectRight,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight,
            const nav_msgs::msg::Odometry::ConstSharedPtr odom);

private:
  // TODO: document
  std::deque<std::pair<std::shared_ptr<rtabmap::SensorData>, nav_msgs::msg::Odometry::ConstSharedPtr>> received_data_queue_;
  std::map<int, std::shared_ptr<rtabmap::SensorData>> local_descriptors_map_;

  std::shared_ptr<rclcpp::Node> node_;

  unsigned int min_inliers_, nb_robots_, robot_id_, max_queue_size_, nb_local_frames_;

  image_transport::SubscriberFilter image_rect_left_;
  image_transport::SubscriberFilter image_rect_right_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_left_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_right_;
  message_filters::Subscriber<nav_msgs::msg::Odometry>  odometry_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo, nav_msgs::msg::Odometry> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> * sync_policy_;

  rclcpp::Subscription<
      cslam_loop_detection_interfaces::msg::LocalDescriptorsRequest>::
      SharedPtr send_local_descriptors_subscriber_;

  rclcpp::Publisher<cslam_loop_detection_interfaces::msg::
                                      LocalImageDescriptors>::SharedPtr
      local_descriptors_publisher_;

  rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeRGB>::SharedPtr
      keyframe_data_publisher_;

  rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr
      keyframe_odom_publisher_;

  rclcpp::Subscription<
      cslam_loop_detection_interfaces::msg::LocalImageDescriptors>::SharedPtr
      local_descriptors_subscriber_;

  rtabmap::RegistrationVis registration_;

  rclcpp::Publisher<cslam_loop_detection_interfaces::msg::
                                      InterRobotLoopClosure>::SharedPtr
      inter_robot_loop_closure_publisher_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_frame_id_;
};
}
#endif