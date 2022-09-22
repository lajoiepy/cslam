#include "cslam/front_end/rgbd_handler.h"
#include "cslam/front_end/sensor_msg_utils.h"

using namespace rtabmap;
using namespace cslam;

RGBDHandler::RGBDHandler(std::shared_ptr<rclcpp::Node> &node)
    : node_(node) {
  node_->declare_parameter<std::string>("frontend.color_image_topic", "color/image");
  node_->declare_parameter<std::string>("frontend.depth_image_topic", "depth/image");
  node_->declare_parameter<std::string>("frontend.color_camera_info_topic",
                                        "color/camera_info");
  node->declare_parameter<std::string>("frontend.odom_topic", "odom");
  node->declare_parameter<float>("frontend.keyframe_generation_ratio_threshold", 0.0);
  node->declare_parameter<std::string>("frontend.sensor_base_frame_id", "camera_link");
  node_->get_parameter("frontend.max_keyframe_queue_size", max_queue_size_);
  node_->get_parameter("frontend.keyframe_generation_ratio_threshold", keyframe_generation_ratio_threshold_);
  node_->get_parameter("frontend.sensor_base_frame_id", base_frame_id_);

  if (keyframe_generation_ratio_threshold_ > 0.99) {
    generate_new_keyframes_based_on_inliers_ratio_ = false;
  } else {
    generate_new_keyframes_based_on_inliers_ratio_ = true;
  }

  nb_local_keyframes_ = 0;
      
  sub_odometry_.subscribe(node_.get(),
                      node_->get_parameter("frontend.odom_topic").as_string(),
                      rclcpp::QoS(max_queue_size_)
                          .reliability((rmw_qos_reliability_policy_t)2)
                          .get_rmw_qos_profile());

  // Service to extract and publish local image descriptors to another robot
  send_local_descriptors_subscriber_ = node_->create_subscription<
      cslam_loop_detection_interfaces::msg::LocalDescriptorsRequest>(
      "local_descriptors_request", 100,
      std::bind(&RGBDHandler::local_descriptors_request, this,
                std::placeholders::_1));

  // Parameters
  node_->get_parameter("frontend.max_keyframe_queue_size", max_queue_size_);
  node_->get_parameter("frontend.pnp_min_inliers", min_inliers_);
  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);

  // Publisher for global descriptors
  keyframe_data_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::KeyframeRGB>(
          "keyframe_data", 100);

  // Publisher for odometry with ID
  keyframe_odom_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::KeyframeOdom>(
          "keyframe_odom", 100);

  // Local matches subscription
  local_keyframe_match_subscriber_ = node->create_subscription<
      cslam_loop_detection_interfaces::msg::LocalKeyframeMatch>(
      "local_keyframe_match", 100,
      std::bind(&RGBDHandler::receive_local_keyframe_match, this,
                std::placeholders::_1));

  // Publishers to other robots local descriptors subscribers
  std::string local_descriptors_topic = "/local_descriptors";
  local_descriptors_publisher_ = node_->create_publisher<
      cslam_loop_detection_interfaces::msg::LocalImageDescriptors>(local_descriptors_topic, 100);
  std::string viz_topic = "/viz/local_descriptors";
  visualization_local_descriptors_publisher_ = node_->create_publisher<
      cslam_loop_detection_interfaces::msg::LocalImageDescriptors>(viz_topic, 100);

  // Subscriber for local descriptors
  local_descriptors_subscriber_ = node->create_subscription<
      cslam_loop_detection_interfaces::msg::LocalImageDescriptors>(
      "/local_descriptors", 100,
      std::bind(&RGBDHandler::receive_local_image_descriptors, this,
                std::placeholders::_1));

  // Registration settings
  rtabmap::ParametersMap registration_params;
  registration_params.insert(rtabmap::ParametersPair(
      rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
  registration_.parseParameters(registration_params);

  // Intra-robot loop closure publisher
  intra_robot_loop_closure_publisher_ = node_->create_publisher<
      cslam_loop_detection_interfaces::msg::IntraRobotLoopClosure>(
      "intra_robot_loop_closure", 100);

  // Publisher for inter robot loop closure to all robots
  inter_robot_loop_closure_publisher_ = node_->create_publisher<
      cslam_loop_detection_interfaces::msg::InterRobotLoopClosure>(
      "/inter_robot_loop_closure", 100);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriber for RGBD images
  sub_image_color_.subscribe(
      node_.get(), node_->get_parameter("frontend.color_image_topic").as_string(), "raw",
      rclcpp::QoS(max_queue_size_)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  sub_image_depth_.subscribe(
      node_.get(), node_->get_parameter("frontend.depth_image_topic").as_string(), "raw",
      rclcpp::QoS(max_queue_size_)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  sub_camera_info_color_.subscribe(
      node_.get(), node_->get_parameter("frontend.color_camera_info_topic").as_string(),
      rclcpp::QoS(max_queue_size_)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());

  rgbd_sync_policy_ = new message_filters::Synchronizer<RGBDSyncPolicy>(
      RGBDSyncPolicy(max_queue_size_), sub_image_color_, sub_image_depth_,
      sub_camera_info_color_, sub_odometry_);
  rgbd_sync_policy_->registerCallback(
      std::bind(&RGBDHandler::rgbd_callback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4));
}

void RGBDHandler::rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr image_rect_rgb,
    const sensor_msgs::msg::Image::ConstSharedPtr image_rect_depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_rgb,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  int image_width = image_rect_rgb->width;
  int image_height = image_rect_rgb->height;
  int depth_width = image_rect_depth->width;
  int depth_height = image_rect_depth->height;
  
  if (!(image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
        image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
        image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
        image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
        image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
        image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
        image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
        image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0) ||
      !(image_rect_depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
        image_rect_depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
        image_rect_depth->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
  {
    RCLCPP_ERROR(node_->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8 and "
                                      "image_depth=32FC1,16UC1,mono16. Current rgb=%s and depth=%s",
                 image_rect_rgb->encoding.c_str(),
                 image_rect_depth->encoding.c_str());
    return;
  }
  
  rclcpp::Time stamp = rtabmap_ros::timestampFromROS(image_rect_rgb->header.stamp) > rtabmap_ros::timestampFromROS(image_rect_depth->header.stamp) ? image_rect_rgb->header.stamp : image_rect_depth->header.stamp;

  Transform local_transform = rtabmap_ros::getTransform(base_frame_id_, image_rect_rgb->header.frame_id, stamp, *tf_buffer_, 0.1);
  if (local_transform.isNull())
  {
    return;
  }

  cv_bridge::CvImageConstPtr ptr_image = cv_bridge::toCvShare(image_rect_rgb);
  if (image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) != 0 &&
      image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
  {
    if (image_rect_rgb->encoding.compare(sensor_msgs::image_encodings::MONO16) != 0)
    {
      ptr_image = cv_bridge::cvtColor(ptr_image, "bgr8");
    }
    else
    {
      ptr_image = cv_bridge::cvtColor(ptr_image, "mono8");
    }
  }

  cv_bridge::CvImageConstPtr ptr_depth = cv_bridge::toCvShare(image_rect_depth);

  CameraModel camera_model = rtabmap_ros::cameraModelFromROS(*camera_info_rgb, local_transform);

  // copy data
  cv::Mat rgb, depth;
  ptr_image->image.copyTo(rgb);
  ptr_depth->image.copyTo(depth);

  auto data = std::make_shared<rtabmap::SensorData>(
      rgb, depth,
      camera_model,
      0,
      rtabmap_ros::timestampFromROS(stamp));

  received_data_queue_.push_back(std::make_pair(data, odom));
  if (received_data_queue_.size() > max_queue_size_)
  {
    // Remove the oldest keyframes if we exceed the maximum size
    received_data_queue_.pop_front();
    RCLCPP_WARN(
        node_->get_logger(),
        "Maximum queue size (%d) exceeded, the oldest element was removed.",
        max_queue_size_);
  }
}

void RGBDHandler::compute_local_descriptors(
    std::shared_ptr<rtabmap::SensorData> &frame_data) {
  // Extract local descriptors
  frame_data->uncompressData();
  std::vector<cv::KeyPoint> kpts_from;
  cv::Mat image = frame_data->imageRaw();
  if (image.channels() > 1) {
    cv::Mat tmp;
    cv::cvtColor(image, tmp, cv::COLOR_BGR2GRAY);
    image = tmp;
  }

  cv::Mat depth_mask;
  if (!frame_data->depthRaw().empty()) {
    if (image.rows % frame_data->depthRaw().rows == 0 &&
        image.cols % frame_data->depthRaw().cols == 0 &&
        image.rows / frame_data->depthRaw().rows ==
            frame_data->imageRaw().cols / frame_data->depthRaw().cols) {
      depth_mask = rtabmap::util2d::interpolate(
          frame_data->depthRaw(),
          frame_data->imageRaw().rows / frame_data->depthRaw().rows, 0.1f);
    } else {
      UWARN("%s is true, but RGB size (%dx%d) modulo depth size (%dx%d) is "
            "not 0. Ignoring depth mask for feature detection.",
            rtabmap::Parameters::kVisDepthAsMask().c_str(),
            frame_data->imageRaw().rows, frame_data->imageRaw().cols,
            frame_data->depthRaw().rows, frame_data->depthRaw().cols);
    }
  }

  rtabmap::ParametersMap registration_params;
  registration_params.insert(rtabmap::ParametersPair(
      rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
  auto detector = rtabmap::Feature2D::create(registration_params);

  auto kpts = detector->generateKeypoints(image, depth_mask);
  auto descriptors = detector->generateDescriptors(image, kpts);
  auto kpts3D = detector->generateKeypoints3D(*frame_data, kpts);

  frame_data->setFeatures(kpts, kpts3D, descriptors);

  // Clear costly data
  frame_data->clearCompressedData();
  frame_data->clearRawData();
}

bool RGBDHandler::generate_new_keyframe(std::shared_ptr<rtabmap::SensorData> & keyframe) {
  // Keyframe generation heuristic
  bool generate_new_keyframe = true;
  if (generate_new_keyframes_based_on_inliers_ratio_) {
    if (nb_local_keyframes_ > 0)
    {
      try{
        rtabmap::RegistrationInfo reg_info;
        rtabmap::Transform t = registration_.computeTransformation(
            *keyframe, *previous_keyframe_, rtabmap::Transform(), &reg_info);
        if (!t.isNull()){
          if(float(reg_info.inliers) >
              keyframe_generation_ratio_threshold_ *
                  float(previous_keyframe_->keypoints().size())) {
            generate_new_keyframe = false;
          }
        }
      }
      catch (std::exception &e) {
        RCLCPP_ERROR(
              node_->get_logger(),
              "Could not compute transformation for keyframe generation: %s",
              e.what());
      }
    }
    if (generate_new_keyframe){
      previous_keyframe_ = keyframe;
    }
  }
  if (generate_new_keyframe){
    // Store descriptors
    keyframe->setId(nb_local_keyframes_);
    local_descriptors_map_.insert({keyframe->id(), keyframe});
    // Setup for next one
    nb_local_keyframes_++;
  }
  return generate_new_keyframe;
}

void RGBDHandler::process_new_sensor_data() {
  if (!received_data_queue_.empty()) {
    auto sensor_data = received_data_queue_.front();
    received_data_queue_.pop_front();

    if (sensor_data.first->isValid()) {
      // Save rgb temporarily for visual place recognition
      cv::Mat rgb;
      sensor_data.first->uncompressDataConst(&rgb, 0);
      // Compute local descriptors
      compute_local_descriptors(sensor_data.first);

      if (generate_new_keyframe(sensor_data.first))
      {
        // rtabmap::LaserScan scan;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
        // rtabmap::util3d::laserScanToPointCloud(scan, scan.local_transform());
        // Send keyframe for loop detection
        send_keyframe(rgb, sensor_data);
      }
    }
  }
}

void RGBDHandler::sensor_data_to_rgbd_msg(
    const std::shared_ptr<rtabmap::SensorData> sensor_data,
    rtabmap_ros::msg::RGBDImage &msg_data) {
  rtabmap_ros::msg::RGBDImage data;
  rtabmap_ros::rgbdImageToROS(*sensor_data, msg_data, "camera");
}

void RGBDHandler::local_descriptors_request(
    cslam_loop_detection_interfaces::msg::LocalDescriptorsRequest::
        ConstSharedPtr request) {
  // Fill msg
  cslam_loop_detection_interfaces::msg::LocalImageDescriptors msg;
  sensor_data_to_rgbd_msg(local_descriptors_map_.at(request->image_id),
                          msg.data);
  msg.image_id = request->image_id;
  msg.robot_id = robot_id_;
  msg.matches_robot_id = request->matches_robot_id;
  msg.matches_image_id = request->matches_image_id;

  // Publish local descriptors
  local_descriptors_publisher_->publish(msg);
}

void RGBDHandler::receive_local_keyframe_match(
    cslam_loop_detection_interfaces::msg::LocalKeyframeMatch::ConstSharedPtr
        msg) {
  try{
    auto keyframe0 = local_descriptors_map_.at(msg->keyframe0_id);
    keyframe0->uncompressData();
    auto keyframe1 = local_descriptors_map_.at(msg->keyframe1_id);
    keyframe1->uncompressData();
    rtabmap::RegistrationInfo reg_info;
    rtabmap::Transform t = registration_.computeTransformation(
        *keyframe0, *keyframe1, rtabmap::Transform(), &reg_info);

    cslam_loop_detection_interfaces::msg::IntraRobotLoopClosure lc;
    lc.keyframe0_id = msg->keyframe0_id;
    lc.keyframe1_id = msg->keyframe1_id;
    if (!t.isNull()) {
      lc.success = true;
      rtabmap_ros::transformToGeometryMsg(t, lc.transform);
    } else {
      lc.success = false;
    }
    intra_robot_loop_closure_publisher_->publish(lc);
  }
  catch (std::exception &e) {
    RCLCPP_ERROR(
          node_->get_logger(),
          "Could not compute local transformation between %d and %d: %s",
          msg->keyframe0_id, msg->keyframe1_id,
          e.what());
  }
}

void RGBDHandler::local_descriptors_msg_to_sensor_data(
    const std::shared_ptr<
        cslam_loop_detection_interfaces::msg::LocalImageDescriptors>
        msg,
    rtabmap::SensorData &sensor_data) {
  // Fill descriptors
  rtabmap::CameraModel camera_model =
      rtabmap_ros::cameraModelFromROS(msg->data.rgb_camera_info,
                                      rtabmap::Transform::getIdentity());
  sensor_data = rtabmap::SensorData(
      cv::Mat(), cv::Mat(), camera_model, 0,
      rtabmap_ros::timestampFromROS(msg->data.header.stamp));

  std::vector<cv::KeyPoint> kpts;
  rtabmap_ros::keypointsFromROS(msg->data.key_points, kpts);
  std::vector<cv::Point3f> kpts3D;
  rtabmap_ros::points3fFromROS(msg->data.points, kpts3D);
  auto descriptors = rtabmap::uncompressData(msg->data.descriptors);
  sensor_data.setFeatures(kpts, kpts3D, descriptors);
}

void RGBDHandler::receive_local_image_descriptors(
    const std::shared_ptr<
        cslam_loop_detection_interfaces::msg::LocalImageDescriptors>
        msg) {
  std::deque<int> image_ids;
  for (unsigned int i = 0; i < msg->matches_robot_id.size(); i++) {
    if (msg->matches_robot_id[i] == robot_id_) {
      image_ids.push_back(msg->matches_image_id[i]);
    }
  }

  for (auto local_image_id : image_ids) {
    try {
      rtabmap::SensorData tmp_to;
      local_descriptors_msg_to_sensor_data(msg, tmp_to);

      // Compute transformation
      //  Registration params
      rtabmap::RegistrationInfo reg_info;
      auto tmp_from = local_descriptors_map_.at(local_image_id);
      tmp_from->uncompressData();
      rtabmap::Transform t = registration_.computeTransformation(
          *tmp_from, tmp_to, rtabmap::Transform(), &reg_info);

      // Store using pairs (robot_id, image_id)
      cslam_loop_detection_interfaces::msg::InterRobotLoopClosure lc;
      lc.robot0_id = robot_id_;
      lc.robot0_image_id = local_image_id;
      lc.robot1_id = msg->robot_id;
      lc.robot1_image_id = msg->image_id;
      if (!t.isNull()) {
        lc.success = true;
        rtabmap_ros::transformToGeometryMsg(t, lc.transform);
        inter_robot_loop_closure_publisher_->publish(lc);
      } else {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Could not compute transformation between (%d,%d) and (%d,%d): %s",
            robot_id_, local_image_id, msg->robot_id, msg->image_id,
            reg_info.rejectedMsg.c_str());
        lc.success = false;
        inter_robot_loop_closure_publisher_->publish(lc);
      }
    } catch (std::exception &e) {
      RCLCPP_ERROR(
            node_->get_logger(),
            "Could not compute transformation between (%d,%d) and (%d,%d): %s",
            robot_id_, local_image_id, msg->robot_id, msg->image_id,
            e.what());
    }
    
  }
}

void RGBDHandler::send_keyframe(const rtabmap::SensorData &rgb,
                     const std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<const nav_msgs::msg::Odometry>>& keypoints_data) {
  // Image message
  std_msgs::msg::Header header;
  header.stamp = node_->now();
  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::RGB8, rgb.imageRaw());
  cslam_common_interfaces::msg::KeyframeRGB keyframe_msg;
  image_bridge.toImageMsg(keyframe_msg.image);
  keyframe_msg.id = keypoints_data.first->id();

  keyframe_data_publisher_->publish(keyframe_msg);

  // Odometry message
  cslam_common_interfaces::msg::KeyframeOdom odom_msg;
  odom_msg.id = keypoints_data.first->id();
  odom_msg.odom = *keypoints_data.second;
  keyframe_odom_publisher_->publish(odom_msg);

  // visualization message
  if (visualization_local_descriptors_publisher_->get_subscription_count() > 0)
  {
    cslam_loop_detection_interfaces::msg::LocalImageDescriptors features_msg;
    sensor_data_to_rgbd_msg(keypoints_data.first,
                            features_msg.data);
    features_msg.image_id = keypoints_data.first->id();
    features_msg.robot_id = robot_id_;
    features_msg.data.key_points.clear();

    // Publish local descriptors
    visualization_local_descriptors_publisher_->publish(features_msg);
  }
}