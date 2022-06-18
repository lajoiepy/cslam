#include "cslam/MapManager.h"

void MapManager::init(std::shared_ptr<rclcpp::Node> &node) {
  node_ = node;

  // Service to add a link in the local pose graph
  // add_link_srv_ = node_->create_client<rtabmap_ros::srv::AddLink>("add_link");
  // while (!add_link_srv_->wait_for_service(std::chrono::seconds(1))) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
  //                  "Interrupted while waiting for the service. Exiting.");
  //     return;
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //               "service not available, waiting again...");
  // }
  nb_local_frames_ = 0;
  // Subscriber for local descriptors
  rgbd_subscriber_ = node->create_subscription<
      rtabmap_ros::msg::RGBDImage>(
      "odom_rgbd_image", 100,
      std::bind(&MapManager::receiveRGBD, this,
                std::placeholders::_1));

  // Service to extract and publish local image descriptors to another robot
  send_local_descriptors_srv_ = node_->create_service<
      cslam_loop_detection_interfaces::srv::SendLocalImageDescriptors>(
      "send_local_image_descriptors",
      std::bind(&MapManager::sendLocalImageDescriptors, this,
                std::placeholders::_1, std::placeholders::_2));

  // Parameters
  node_->get_parameter("max_keyframe_queue_size", max_queue_size_);
  node_->get_parameter("pnp_min_inliers", min_inliers_);
  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);

  // Publisher for global descriptors
  keyframe_data_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::KeyframeRGB>(
          "keyframe_data", 100);

  // Publisher to all robots inter robot loop closure subscribers
  for (unsigned int id = 0; id < nb_robots_; id++) {
    std::string topic = "/r" + std::to_string(id) + "/inter_robot_loop_closure";
    inter_robot_loop_closure_publishers_.insert(
        {id, node_->create_publisher<
                 cslam_loop_detection_interfaces::msg::InterRobotLoopClosure>(
                 topic, 100)});
  }

  // Publishers to other robots local descriptors subscribers
  for (unsigned int id = 0; id < nb_robots_; id++) {
    if (id != robot_id_) {
      std::string topic = "/r" + std::to_string(id) + "/local_descriptors";
      local_descriptors_publishers_.insert(
          {id, node_->create_publisher<
                   cslam_loop_detection_interfaces::msg::LocalImageDescriptors>(
                   topic, 100)});
    }
  }

  // Subscriber for local descriptors
  local_descriptors_subscriber_ = node->create_subscription<
      cslam_loop_detection_interfaces::msg::LocalImageDescriptors>(
      "local_descriptors", 100,
      std::bind(&MapManager::receiveLocalImageDescriptors, this,
                std::placeholders::_1));

  // Registration settings
  rtabmap::ParametersMap registration_params;
  registration_params.insert(rtabmap::ParametersPair(
      rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
  registration_.parseParameters(registration_params);

  RCLCPP_INFO(node_->get_logger(), "Initialization done.");
}

void MapManager::sendKeyframe(const rtabmap::SensorData &data,
                                  const int id) {
  RCLCPP_INFO(node_->get_logger(),
              "Process Image %d for Loop Closure Detection", id);
  // Image message
  std_msgs::msg::Header header;
  header.stamp = node_->now();
  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::RGB8, data.imageRaw());
  cslam_common_interfaces::msg::KeyframeRGB keyframe_msg;
  image_bridge.toImageMsg(keyframe_msg.image);
  keyframe_msg.id = id;

  keyframe_data_publisher_->publish(keyframe_msg);
}

void MapManager::processNewKeyFrames() {
  if (!received_data_queue_.empty()) {
    auto image_data = received_data_queue_.front();
    received_data_queue_.pop_front();

    rtabmap::SensorData sensor_data = rtabmap_ros::rgbdImageFromROS(image_data);

    if (sensor_data.isValid() &&
        local_data_.find(nb_local_frames_) == local_data_.end()) {
      cv::Mat rgb;
      // rtabmap::LaserScan scan;
      sensor_data.uncompressDataConst(&rgb, 0);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      // rtabmap::util3d::laserScanToPointCloud(scan, scan.localTransform());
      // Send keyframe for loop detection
      sendKeyframe(rgb, nb_local_frames_);

      local_data_.insert(std::make_pair(nb_local_frames_, sensor_data));
      nb_local_frames_++;
    }
  }
}

void MapManager::geometricVerification() {
  // TODO: Use for intra-robot loop closures
  /*int from_id = res.from_id;
  int to_id = res.to_id;
  RCLCPP_INFO(node_->get_logger(), "Detected loop closure between %d and %d",
              from_id, to_id);
  if (local_data_.find(to_id) != local_data_.end()) {
    // Compute transformation
    // Registration params
    rtabmap::RegistrationInfo reg_info;
    rtabmap::SensorData tmp_from = local_data_.at(from_id);
    rtabmap::SensorData tmp_to = local_data_.at(to_id);
    tmp_from.uncompressData();
    tmp_to.uncompressData();
    rtabmap::Transform t = registration_.computeTransformation(
        tmp_from, tmp_to, rtabmap::Transform(), &reg_info);

    if (!t.isNull()) { // TODO: Only if intra-robot loop closure
      rtabmap::Link link(from_id, to_id, rtabmap::Link::kUserClosure, t,
                         reg_info.covariance.inv());
      auto request = std::make_shared<rtabmap_ros::srv::AddLink::Request>();
      rtabmap_ros::linkToROS(link, request->link);
      auto result = add_link_srv_->async_send_request(request);
      « RCLCPP_INFO(node_->get_logger(), "Add link service called");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Could not compute transformation between %d and %d: %s",
                  from_id, to_id, reg_info.rejectedMsg.c_str());
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "Could not compute transformation between %d and %d "
                "because node data %d is not in cache.",
                from_id, to_id, to_id);
  }*/
}

void MapManager::receiveRGBD(
    const std::shared_ptr<rtabmap_ros::msg::RGBDImage> image_msg) {
  RCLCPP_INFO(node_->get_logger(), "Received image!");

  // TODO: Add keyframe heuristic

  received_data_queue_.push_back(image_msg);
  if (received_data_queue_.size() > max_queue_size_) {
    // Remove the oldest keyframes if we exceed the maximum size
    received_data_queue_.pop_front();
    RCLCPP_WARN(
        node_->get_logger(),
        "Maximum queue size (%d) exceeded, the oldest element was removed.",
        max_queue_size_);
  }
}

void MapManager::sendLocalImageDescriptors(
    const std::shared_ptr<cslam_loop_detection_interfaces::srv::
                              SendLocalImageDescriptors::Request>
        request,
    std::shared_ptr<cslam_loop_detection_interfaces::srv::
                        SendLocalImageDescriptors::Response>
        response) {
  // Extract local descriptors
  rtabmap::SensorData frame_data = local_data_.at(request->image_id);
  frame_data.uncompressData();
  std::vector<cv::KeyPoint> kpts_from;
  cv::Mat image = frame_data.imageRaw();
  if (image.channels() > 1) {
    cv::Mat tmp;
    cv::cvtColor(image, tmp, cv::COLOR_BGR2GRAY);
    image = tmp;
  }

  cv::Mat depth_mask;
  if (!frame_data.depthRaw().empty()) {
    if (image.rows % frame_data.depthRaw().rows == 0 &&
        image.cols % frame_data.depthRaw().cols == 0 &&
        image.rows / frame_data.depthRaw().rows ==
            frame_data.imageRaw().cols / frame_data.depthRaw().cols) {
      depth_mask = rtabmap::util2d::interpolate(
          frame_data.depthRaw(),
          frame_data.imageRaw().rows / frame_data.depthRaw().rows, 0.1f);
    } else {
      UWARN("%s is true, but RGB size (%dx%d) modulo depth size (%dx%d) is "
            "not 0. Ignoring depth mask for feature detection.",
            rtabmap::Parameters::kVisDepthAsMask().c_str(),
            frame_data.imageRaw().rows, frame_data.imageRaw().cols,
            frame_data.depthRaw().rows, frame_data.depthRaw().cols);
    }
  }

  rtabmap::ParametersMap registration_params;
  registration_params.insert(rtabmap::ParametersPair(
      rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
  auto detector = rtabmap::Feature2D::create(registration_params);

  auto kpts = detector->generateKeypoints(image, depth_mask);
  auto descriptors = detector->generateDescriptors(image, kpts);
  auto kpts3D = detector->generateKeypoints3D(frame_data, kpts);

  // Build message
  frame_data.setFeatures(kpts, kpts3D, descriptors);
  rtabmap_ros::msg::RGBDImage data;
  rtabmap_ros::rgbdImageToROS(frame_data, data, "camera");

  // Clear images in message to save bandwidth
  data.rgb = sensor_msgs::msg::Image();
  data.depth = sensor_msgs::msg::Image();
  data.rgb_compressed = sensor_msgs::msg::CompressedImage();
  data.depth_compressed = sensor_msgs::msg::CompressedImage();
  data.global_descriptor = rtabmap_ros::msg::GlobalDescriptor();

  // Fill msg
  cslam_loop_detection_interfaces::msg::LocalImageDescriptors msg;
  msg.data = data;
  msg.image_id = request->image_id;
  msg.robot_id = robot_id_;
  msg.receptor_image_id = request->receptor_image_id;

  // Publish local descriptors
  local_descriptors_publishers_.at(request->receptor_robot_id)->publish(msg);

  response->success = true;
}

void MapManager::receiveLocalImageDescriptors(
    const std::shared_ptr<
        cslam_loop_detection_interfaces::msg::LocalImageDescriptors>
        msg) {
  // Fill keypoints
  rtabmap::StereoCameraModel stereo_model =
      rtabmap_ros::stereoCameraModelFromROS(msg->data.rgb_camera_info,
                                            msg->data.depth_camera_info,
                                            rtabmap::Transform::getIdentity());
  rtabmap::SensorData tmp_to(
      cv::Mat(), cv::Mat(), stereo_model, 0,
      rtabmap_ros::timestampFromROS(msg->data.header.stamp));

  std::vector<cv::KeyPoint> kpts;
  rtabmap_ros::keypointsFromROS(msg->data.key_points, kpts);
  std::vector<cv::Point3f> kpts3D;
  rtabmap_ros::points3fFromROS(msg->data.points, kpts3D);
  auto descriptors = rtabmap::uncompressData(msg->data.descriptors);
  tmp_to.setFeatures(kpts, kpts3D, descriptors);

  // Compute transformation
  //  Registration params
  rtabmap::RegistrationInfo reg_info;
  rtabmap::SensorData tmp_from = local_data_.at(msg->receptor_image_id);
  tmp_from.uncompressData();
  rtabmap::Transform t = registration_.computeTransformation(
      tmp_from, tmp_to, rtabmap::Transform(), &reg_info);

  // Store using pairs (robot_id, image_id)
  cslam_loop_detection_interfaces::msg::InterRobotLoopClosure lc;
  lc.robot0_id = robot_id_;
  lc.robot0_image_id = msg->receptor_image_id;
  lc.robot1_id = msg->robot_id;
  lc.robot1_image_id = msg->image_id;
  if (!t.isNull()) {
    RCLCPP_INFO(node_->get_logger(), "Inter-Robot Link computed");
    lc.success = true;
    rtabmap_ros::transformToGeometryMsg(t, lc.transform);
    inter_robot_loop_closure_publishers_[lc.robot0_id]->publish(lc);
    inter_robot_loop_closure_publishers_[lc.robot1_id]->publish(lc);
  } else {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Could not compute transformation between (%d,%d) and (%d,%d): %s",
        robot_id_, msg->receptor_image_id, msg->robot_id, msg->image_id,
        reg_info.rejectedMsg.c_str());
    lc.success = false;
    inter_robot_loop_closure_publishers_[lc.robot0_id]->publish(lc);
    inter_robot_loop_closure_publishers_[lc.robot1_id]->publish(lc);
  }
}
