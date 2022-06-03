#include <rclcpp/rclcpp.hpp>

#include <rtabmap_ros/msg/info.hpp>
#include <rtabmap_ros/msg/map_data.hpp>
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
#include <cslam_loop_detection/msg/local_image_descriptors.hpp>
#include <cslam_loop_detection/srv/detect_loop_closure.hpp>
#include <cslam_loop_detection/srv/send_local_image_descriptors.hpp>
#include <cslam_utils/msg/image_id.hpp>
#include <deque>
#include <functional>
#include <thread>

// Message filters to sync callbacks
typedef message_filters::sync_policies::ExactTime<rtabmap_ros::msg::map_data,
                                                  rtabmap_ros::msg::Info>
    MyInfoMapSyncPolicy;

// Response
struct LoopClosureResponse {
  bool is_valid;
  bool is_detected;
  int from_id;
  int to_id;
};

// Use an external service for loop closure detection
class ExternalLoopClosureService {
public:
  ExternalLoopClosureService(){};

  ~ExternalLoopClosureService(){};

  void init(std::shared_ptr<rclcpp::Node> &node) {
    node_ = node;
    client_ =
        node_->create_client<cslam_loop_detection::srv::DetectLoopClosure>(
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

  void detectLoopClosures(const rtabmap::SensorData &data, const int id) {
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
    auto request = std::make_shared<
        cslam_loop_detection::srv::DetectLoopClosure::Request>();
    request->image = image_msg;

    responses_.push_back(client_->async_send_request(request));
    RCLCPP_INFO(node_->get_logger(),
                "Service called for place recognition processing");
  }

  LoopClosureResponse checkForResponse() {
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

private:
  rclcpp::Client<cslam_loop_detection::srv::DetectLoopClosure>::SharedPtr
      client_;
  std::deque<rclcpp::Client<
      cslam_loop_detection::srv::DetectLoopClosure>::SharedFuture>
      responses_;
  std::shared_ptr<rclcpp::Node> node_;
};

class ExternalLoopClosureDetection {
public:
  ExternalLoopClosureDetection(){};
  ~ExternalLoopClosureDetection(){};

  void init(std::shared_ptr<rclcpp::Node> &node) {
    node_ = node;
    loop_closure_detector_.init(node_);

    // Service to add a link in the local pose graph
    std::string add_link_srv;
    node_->get_parameter("AddLinkSrv", add_link_srv);
    add_link_srv_ =
        node_->create_client<rtabmap_ros::srv::AddLink>(add_link_srv);
    while (!add_link_srv_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    // Service to extract and publish local image descriptors to another robot
    send_local_descriptors_srv_ = node_->create_service<
        cslam_loop_detection::srv::SendLocalImageDescriptors>(
        "send_local_image_descriptors",
        std::bind(&ExternalLoopClosureDetection::send_local_image_descriptors,
                  this, std::placeholders::_1, std::placeholders::_2));

    // Parameters
    node_->get_parameter("max_queue_size", max_queue_size_);
    node_->get_parameter("min_inliers", min_inliers_);
    node_->get_parameter("number_of_robots", nb_robots_);
    node_->get_parameter("robot_id", robot_id_);

    // Publishers to other robots local descriptors subscribers
    for (int id = 0; id < nb_robots_; id++) {
      if (id != robot_id_) {
        std::string topic = "/r" + std::to_string(id) + "/local_descriptors";
        local_descriptors_publishers_.insert(
            {id,
             node_->create_publisher<
                 cslam_loop_detection::msg::LocalImageDescriptors>(topic, 10)});
      }
    }

    // Subscriber for local descriptors
    local_descriptors_subscriber_ = node->create_subscription<
        cslam_loop_detection::msg::LocalImageDescriptors>(
        "local_descriptors", 10,
        std::bind(
            &ExternalLoopClosureDetection::receive_local_image_descriptors,
            this, std::placeholders::_1));

    // Registration settings
    rtabmap::ParametersMap registration_params;
    registration_params.insert(rtabmap::ParametersPair(
        rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
    registration_.parseParameters(registration_params);

    RCLCPP_INFO(node_->get_logger(), "Initialization done.");
  }

  void processNewKeyFrames() {
    if (!received_data_queue_.empty()) {
      auto map_data = received_data_queue_.front();
      received_data_queue_.pop_front();

      rtabmap::Transform map_to_odom;
      std::map<int, rtabmap::Transform> poses;
      std::multimap<int, rtabmap::Link> links;
      std::map<int, rtabmap::Signature> signatures;
      rtabmap_ros::map_dataFromROS(*map_data, poses, links, signatures,
                                   map_to_odom);

      if (!signatures.empty() &&
          signatures.rbegin()->second.sensorData().isValid() &&
          local_data_.find(signatures.rbegin()->first) == local_data_.end()) {
        int id = signatures.rbegin()->first;
        const rtabmap::SensorData &s = signatures.rbegin()->second.sensorData();
        cv::Mat rgb;
        // rtabmap::LaserScan scan;
        s.uncompressDataConst(&rgb, 0);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
        // rtabmap::util3d::laserScanToPointCloud(scan, scan.localTransform());
        //  Send request for loop detection
        loop_closure_detector_.detectLoopClosures(rgb, id);

        local_data_.insert(std::make_pair(id, s));
      }
    }
  }

  void processLoopClosure() {
    LoopClosureResponse res = loop_closure_detector_.checkForResponse();
    if (res.is_valid) {
      if (res.is_detected) {
        int from_id = res.from_id;
        int to_id = res.to_id;
        RCLCPP_INFO(node_->get_logger(),
                    "Detected loop closure between %d and %d", from_id, to_id);
        if (local_data_.find(to_id) != local_data_.end()) {
          // Compute transformation
          //  Registration params
          rtabmap::RegistrationInfo reg_info;
          rtabmap::SensorData tmp_from = local_data_.at(from_id);
          rtabmap::SensorData tmp_to = local_data_.at(to_id);
          tmp_from.uncompressData();
          tmp_to.uncompressData();
          rtabmap::Transform t = registration_.computeTransformation(
              tmp_from, tmp_to, rtabmap::Transform(), &reg_info);

          if (!t.isNull()) {
            rtabmap::Link link(from_id, to_id, rtabmap::Link::kUserClosure, t,
                               reg_info.covariance.inv());
            auto request =
                std::make_shared<rtabmap_ros::srv::AddLink::Request>();
            rtabmap_ros::linkToROS(link, request->link);
            auto result = add_link_srv_->async_send_request(request);
            // Do not need to wait for response
            // if(!(rclcpp::spin_until_future_complete(node_, result) ==
            // 	rclcpp::FutureReturnCode::SUCCESS))
            // {
            // 	RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            // }
            RCLCPP_INFO(node_->get_logger(), "Add link service called");
          } else {
            RCLCPP_WARN(
                node_->get_logger(),
                "Could not compute transformation between %d and %d: %s",
                from_id, to_id, reg_info.rejectedMsg.c_str());
          }
        } else {
          RCLCPP_WARN(node_->get_logger(),
                      "Could not compute transformation between %d and %d "
                      "because node data %d is not in cache.",
                      from_id, to_id, to_id);
        }
      }
    }
  }

  void mapDataCallback(
      const std::shared_ptr<rtabmap_ros::msg::map_data> &map_data_msg,
      const std::shared_ptr<rtabmap_ros::msg::Info> &info_msg) {
    RCLCPP_INFO(node_->get_logger(), "Received map data!");

    rtabmap::Statistics stats;
    rtabmap_ros::infoFromROS(*info_msg, stats);

    bool small_movement = (bool)uValue(
        stats.data(), rtabmap::Statistics::kMemorySmall_movement(), 0.0f);
    bool fast_movement = (bool)uValue(
        stats.data(), rtabmap::Statistics::kMemoryFast_movement(), 0.0f);

    if (small_movement || fast_movement) {
      // The signature has been ignored from rtabmap, don't process it
      RCLCPP_INFO(node_->get_logger(),
                  "Ignore keyframe. Small movement=%d, Fast movement=%d",
                  (int)small_movement, (int)fast_movement);
      return;
    }

    received_data_queue_.push_back(map_data_msg);
    if (received_data_queue_.size() > max_queue_size_) {
      // Remove the oldest keyframes if we exceed the maximum size
      received_data_queue_.pop_front();
      RCLCPP_WARN(
          node_->get_logger(),
          "Maximum queue size (%d) exceeded, the oldest element was removed.",
          max_queue_size_);
    }
  }

  void send_local_image_descriptors(
      const std::shared_ptr<
          cslam_loop_detection::srv::SendLocalImageDescriptors::Request>
          request,
      std::shared_ptr<
          cslam_loop_detection::srv::SendLocalImageDescriptors::Response>
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
    cslam_loop_detection::msg::LocalImageDescriptors msg;
    msg.data = data;
    msg.image_id = request->image_id;
    msg.robot_id = robot_id_;
    msg.receptor_image_id = request->receptor_image_id;

    // Publish local descriptors
    local_descriptors_publishers_.at(request->receptor_robot_id)->publish(msg);

    response->success = true;
  }

  void receive_local_image_descriptors(
      const std::shared_ptr<cslam_loop_detection::msg::LocalImageDescriptors>
          msg) {
    // Fill keypoints
    rtabmap::StereoCameraModel stereo_model =
        rtabmap_ros::stereoCameraModelFromROS(
            msg->data.rgb_camera_info, msg->data.depth_camera_info,
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
    if (!t.isNull()) {
      RCLCPP_ERROR(node_->get_logger(), "Inter-Robot Link computed");
    } else {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Could not compute transformation between (%d,%d) and (%d,%d): %s",
          robot_id_, msg->receptor_image_id, msg->robot_id, msg->image_id,
          reg_info.rejectedMsg.c_str());
    }
  }

private:
  ExternalLoopClosureService loop_closure_detector_;

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

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("external_loop_closure_detection");

  node->declare_parameter<std::string>("add_link_srv", "/rtabmap/add_link");
  node->declare_parameter<std::string>("rtabmap_info_topic", "/rtabmap/info");
  node->declare_parameter<std::string>("rtabmap_map_topic",
                                       "/rtabmap/map_data");
  node->declare_parameter<int>("min_inliers", 20);
  node->declare_parameter<int>("max_queue_size", 10);
  node->declare_parameter<int>("number_of_robots", 1);
  node->declare_parameter<int>("robot_id", 0);

  message_filters::Subscriber<rtabmap_ros::msg::Info> info_topic_;

  message_filters::Subscriber<rtabmap_ros::msg::map_data> map_data_topic_;

  message_filters::Synchronizer<MyInfoMapSyncPolicy> *info_map_sync_;

  std::string info_topic;
  node->get_parameter("rtabmap_info_topic", info_topic);
  std::string map_topic;
  node->get_parameter("rtabmap_map_topic", map_topic);

  info_topic_.subscribe(node.get(), info_topic);
  map_data_topic_.subscribe(node.get(), map_topic);
  info_map_sync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(
      MyInfoMapSyncPolicy(10), map_data_topic_, info_topic_);

  auto lcd = ExternalLoopClosureDetection();
  lcd.init(node);

  info_map_sync_->registerCallback(
      &ExternalLoopClosureDetection::mapDataCallback, &lcd);

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    lcd.processNewKeyFrames();
    lcd.processLoopClosure();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
