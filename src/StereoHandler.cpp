#include "cslam/StereoHandler.h"
#include "cslam/MsgConversion.h"

using namespace rtabmap;

StereoHandler::StereoHandler(std::shared_ptr<rclcpp::Node> &node): node_(node){
    node->declare_parameter<std::string>("left_image_topic", "left/image_rect");
    node->declare_parameter<std::string>("right_image_topic", "right/image_rect");
    node->declare_parameter<std::string>("left_camera_info_topic", "left/camera_info");
    node->declare_parameter<std::string>("right_camera_info_topic", "right/camera_info");
    node_->get_parameter("max_keyframe_queue_size", max_queue_size_);

    nb_local_frames_ = 0;
    base_frame_id_ = "camera_link"; // TODO: add param

    // Subscriber for stereo images
    int queue_size = 10; // TODO: param
    imageRectLeft_.subscribe(node_.get(), node_->get_parameter("left_image_topic").as_string(), "raw", rclcpp::QoS(queue_size).reliability((rmw_qos_reliability_policy_t)2).get_rmw_qos_profile());
    imageRectRight_.subscribe(node_.get(), node_->get_parameter("right_image_topic").as_string(), "raw", rclcpp::QoS(queue_size).reliability((rmw_qos_reliability_policy_t)2).get_rmw_qos_profile());
    cameraInfoLeft_.subscribe(node_.get(), node_->get_parameter("left_camera_info_topic").as_string(), rclcpp::QoS(queue_size).reliability((rmw_qos_reliability_policy_t)2).get_rmw_qos_profile());
    cameraInfoRight_.subscribe(node_.get(), node_->get_parameter("right_camera_info_topic").as_string(), rclcpp::QoS(queue_size).reliability((rmw_qos_reliability_policy_t)2).get_rmw_qos_profile());

    exactSync_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queue_size), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
    exactSync_->registerCallback(std::bind(&StereoHandler::stereo_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    
    // Service to extract and publish local image descriptors to another robot
    send_local_descriptors_srv_ = node_->create_service<
        cslam_loop_detection_interfaces::srv::SendLocalImageDescriptors>(
        "send_local_image_descriptors",
        std::bind(&StereoHandler::send_local_image_descriptors, this,
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
        std::bind(&StereoHandler::receive_local_image_descriptors, this,
                    std::placeholders::_1));

    // Registration settings
    rtabmap::ParametersMap registration_params;
    registration_params.insert(rtabmap::ParametersPair(
        rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
    registration_.parseParameters(registration_params);


    // Publisher to all robots inter robot loop closure publisher
    for (unsigned int id = 0; id < nb_robots_; id++) {
        std::string topic = "/r" + std::to_string(id) + "/inter_robot_loop_closure";
        inter_robot_loop_closure_publishers_.insert(
            {id, node_->create_publisher<
                    cslam_loop_detection_interfaces::msg::InterRobotLoopClosure>(
                    topic, 100)});
    }

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void StereoHandler::stereo_callback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageRectLeft,
		const sensor_msgs::msg::Image::ConstSharedPtr imageRectRight,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight)
{
    if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
            imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
            imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
            imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
            imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
            imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
            imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) ||
        !(imageRectRight->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
            imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
            imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
            imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
            imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
            imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
            imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0))
    {
        RCLCPP_ERROR(node_->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (mono8 recommended), received types are %s (left) and %s (right)",
                imageRectLeft->encoding.c_str(), imageRectRight->encoding.c_str());
        return;
    }

    rclcpp::Time stamp = rtabmap_ros::timestampFromROS(imageRectLeft->header.stamp)>rtabmap_ros::timestampFromROS(imageRectRight->header.stamp)?imageRectLeft->header.stamp:imageRectRight->header.stamp;

    Transform localTransform = rtabmap_ros::getTransform(base_frame_id_, imageRectLeft->header.frame_id, stamp, *tf_buffer_, 0.1);
    if(localTransform.isNull())
    {
        return;
    }

    if(imageRectLeft->data.size() && imageRectRight->data.size())
    {
        bool alreadyRectified = true;
        rtabmap::Transform stereoTransform;
        if(!alreadyRectified)
        {
            stereoTransform = rtabmap_ros::getTransform(
                    cameraInfoRight->header.frame_id,
                    cameraInfoLeft->header.frame_id,
                    cameraInfoLeft->header.stamp,
                    *tf_buffer_,
                    0.1);
            if(stereoTransform.isNull())
            {
                RCLCPP_ERROR(node_->get_logger(), "Parameter %s is false but we cannot get TF between the two cameras! (between frames %s and %s)",
                        Parameters::kRtabmapImagesAlreadyRectified().c_str(),
                        cameraInfoRight->header.frame_id.c_str(),
                        cameraInfoLeft->header.frame_id.c_str());
                return;
            }
            else if(stereoTransform.isIdentity())
            {
                RCLCPP_ERROR(node_->get_logger(), "Parameter %s is false but we cannot get a valid TF between the two cameras! "
                        "Identity transform returned between left and right cameras. Verify that if TF between "
                        "the cameras is valid: \"rosrun tf tf_echo %s %s\".",
                        Parameters::kRtabmapImagesAlreadyRectified().c_str(),
                        cameraInfoRight->header.frame_id.c_str(),
                        cameraInfoLeft->header.frame_id.c_str());
                return;
            }
        }

        rtabmap::StereoCameraModel stereoModel = rtabmap_ros::stereoCameraModelFromROS(*cameraInfoLeft, *cameraInfoRight, localTransform, stereoTransform);

        if(stereoModel.baseline() == 0 && alreadyRectified)
        {
            stereoTransform = rtabmap_ros::getTransform(
                    cameraInfoLeft->header.frame_id,
                    cameraInfoRight->header.frame_id,
                    cameraInfoLeft->header.stamp,
                    *tf_buffer_,
                    0.1);

            if(!stereoTransform.isNull() && stereoTransform.x()>0)
            {
                static bool warned = false;
                if(!warned)
                {
                    RCLCPP_WARN(node_->get_logger(), "Right camera info doesn't have Tx set but we are assuming that stereo images are already rectified (see %s parameter). While not "
                            "recommended, we used TF to get the baseline (%s->%s = %fm) for convenience (e.g., D400 ir stereo issue). It is preferred to feed "
                            "a valid right camera info if stereo images are already rectified. This message is only printed once...",
                            rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
                            cameraInfoRight->header.frame_id.c_str(), cameraInfoLeft->header.frame_id.c_str(), stereoTransform.x());
                    warned = true;
                }
                stereoModel = rtabmap::StereoCameraModel(
                        stereoModel.left().fx(),
                        stereoModel.left().fy(),
                        stereoModel.left().cx(),
                        stereoModel.left().cy(),
                        stereoTransform.x(),
                        stereoModel.localTransform(),
                        stereoModel.left().imageSize());
            }
        }

        if(alreadyRectified && stereoModel.baseline() <= 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "The stereo baseline (%f) should be positive (baseline=-Tx/fx). We assume a horizontal left/right stereo "
                        "setup where the Tx (or P(0,3)) is negative in the right camera info msg.", stereoModel.baseline());
            return;
        }

        if(stereoModel.baseline() > 10.0)
        {
            static bool shown = false;
            if(!shown)
            {
                RCLCPP_WARN(node_->get_logger(), "Detected baseline (%f m) is quite large! Is your "
                            "right camera_info P(0,3) correctly set? Note that "
                            "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
                            stereoModel.baseline());
                shown = true;
            }
        }

        cv_bridge::CvImagePtr ptrImageLeft = cv_bridge::toCvCopy(imageRectLeft,
                imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
                imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8)==0?"":
                    imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0?"bgr8":"mono8");
        cv_bridge::CvImagePtr ptrImageRight = cv_bridge::toCvCopy(imageRectRight,
                imageRectRight->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
                imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8)==0?"":"mono8");
        
        auto data = std::make_shared<rtabmap::SensorData>(
                ptrImageLeft->image,
                ptrImageRight->image,
                stereoModel,
                nb_local_frames_,
                rtabmap_ros::timestampFromROS(stamp));
            
        received_data_queue_.push_back(data);
        if (received_data_queue_.size() > max_queue_size_) {
            // Remove the oldest keyframes if we exceed the maximum size
            received_data_queue_.pop_front();
            RCLCPP_WARN(
                node_->get_logger(),
                "Maximum queue size (%d) exceeded, the oldest element was removed.",
                max_queue_size_);
        }

        nb_local_frames_++;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Odom: input images empty?!?");
    }
}

void StereoHandler::process_new_keyframe(){
    if (!received_data_queue_.empty()) {
        auto sensor_data = received_data_queue_.front();
        received_data_queue_.pop_front();
        // TODO: keyframe heuristic

        if (sensor_data->isValid() &&
            local_data_map_.find(sensor_data->id()) == local_data_map_.end()) {
            cv::Mat rgb;
            // rtabmap::LaserScan scan;
            sensor_data->uncompressDataConst(&rgb, 0);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
            // rtabmap::util3d::laserScanToPointCloud(scan, scan.localTransform());
            // Send keyframe for loop detection
            send_keyframe(rgb, sensor_data->id());

            local_data_map_.insert(std::make_pair(sensor_data->id(), sensor_data));
        }
    }
}

void StereoHandler::send_local_image_descriptors(
    const std::shared_ptr<cslam_loop_detection_interfaces::srv::
                              SendLocalImageDescriptors::Request>
        request,
    std::shared_ptr<cslam_loop_detection_interfaces::srv::
                        SendLocalImageDescriptors::Response>
        response) {
  // Extract local descriptors
  auto frame_data = local_data_map_.at(request->image_id);
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

  // Build message
  frame_data->setFeatures(kpts, kpts3D, descriptors);
  rtabmap_ros::msg::RGBDImage data;
  rtabmap_ros::rgbdImageToROS(*frame_data, data, "camera");

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

void StereoHandler::receive_local_image_descriptors(
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
  auto tmp_from = local_data_map_.at(msg->receptor_image_id);
  tmp_from->uncompressData();
  rtabmap::Transform t = registration_.computeTransformation(
      *tmp_from, tmp_to, rtabmap::Transform(), &reg_info);

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

void StereoHandler::send_keyframe(const rtabmap::SensorData &data,
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