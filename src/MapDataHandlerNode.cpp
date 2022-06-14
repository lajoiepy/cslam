#include "cslam/MapDataHandler.h"

// Message filters to sync callbacks
typedef message_filters::sync_policies::ExactTime<rtabmap_ros::msg::MapData,
                                                  rtabmap_ros::msg::Info>
    InfoMapSyncPolicy;

/**
 * @brief Node to interface with RTAB-map library for keyframe and 3D features
 * handling
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("map_data_handler");

  node->declare_parameter<std::string>("rtabmap_info_topic", "info");
  node->declare_parameter<std::string>("rtabmap_map_topic",
                                       "mapData");
  node->declare_parameter<int>("pnp_min_inliers", 20);
  node->declare_parameter<int>("max_keyframe_queue_size", 10);
  node->declare_parameter<int>("nb_robots", 1);
  node->declare_parameter<int>("robot_id", 0);
  

  message_filters::Subscriber<rtabmap_ros::msg::Info> info_topic_;

  message_filters::Subscriber<rtabmap_ros::msg::MapData> map_data_topic_;

  message_filters::Synchronizer<InfoMapSyncPolicy> *info_map_sync_;

  std::string info_topic;
  node->get_parameter("rtabmap_info_topic", info_topic);
  std::string map_topic;
  node->get_parameter("rtabmap_map_topic", map_topic);

  info_topic_.subscribe(node.get(), info_topic);
  map_data_topic_.subscribe(node.get(), map_topic);
  info_map_sync_ = new message_filters::Synchronizer<InfoMapSyncPolicy>(
      InfoMapSyncPolicy(10), map_data_topic_, info_topic_);

  auto lcd = MapDataHandler();
  lcd.init(node);

  info_map_sync_->registerCallback(&MapDataHandler::mapDataCallback,
                                   &lcd);

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    lcd.processNewKeyFrames();
    //lcd.geometricVerification();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
