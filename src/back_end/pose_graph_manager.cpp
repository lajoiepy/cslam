#include "cslam/back_end/pose_graph_manager.h"

PoseGraphManager::PoseGraphManager(std::shared_ptr<rclcpp::Node> &node): node_(node) {

  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);
  node_->get_parameter("pose_graph_manager_process_period_ms", pose_graph_manager_process_period_ms_);

  odometry_subscriber_ = node->create_subscription<
        cslam_common_interfaces::msg::KeyframeOdom>(
        "keyframe_odom", 100,
        std::bind(&PoseGraphManager::odometry_callback, this,
                    std::placeholders::_1));

  inter_robot_loop_closure_subscriber_ = node->create_subscription<
        cslam_loop_detection_interfaces::msg::
                                      InterRobotLoopClosure>(
        "inter_robot_loop_closure", 100,
        std::bind(&PoseGraphManager::inter_robot_loop_closure_callback, this,
                    std::placeholders::_1));

  rotation_default_noise_std_ = 0.01;
  translation_default_noise_std_ = 0.1;
  Eigen::VectorXd sigmas(6);
  sigmas << rotation_default_noise_std_, rotation_default_noise_std_, rotation_default_noise_std_, 
            translation_default_noise_std_, translation_default_noise_std_, translation_default_noise_std_;
  default_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  pose_graph_ = boost::make_shared<gtsam::NonlinearFactorGraph>();
  current_pose_estimates_ = boost::make_shared<gtsam::Values>();
}

void PoseGraphManager::odometry_msg_to_pose3(const nav_msgs::msg::Odometry& odom_msg, gtsam::Pose3& pose){
  gtsam::Rot3 rotation(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
  pose = gtsam::Pose3(rotation, {odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z});
}

void PoseGraphManager::transform_msg_to_pose3(const geometry_msgs::msg::Transform& msg, gtsam::Pose3& pose){
  gtsam::Rot3 rotation(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
  pose = gtsam::Pose3(rotation, {msg.translation.x, msg.translation.y, msg.translation.z});
}

void PoseGraphManager::odometry_callback(const cslam_common_interfaces::msg::KeyframeOdom::ConstSharedPtr msg) {

  gtsam::Pose3 current_estimate;
  odometry_msg_to_pose3(msg->odom, current_estimate);
  gtsam::LabeledSymbol symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), msg->id);
  current_pose_estimates_->insert(symbol, current_estimate);

  if (latest_local_symbol_ != gtsam::LabeledSymbol())
  {
    gtsam::Pose3 odom_diff = current_estimate * latest_local_pose_.inverse();
    gtsam::BetweenFactor<gtsam::Pose3> factor(latest_local_symbol_, symbol, odom_diff, default_noise_model_);
    pose_graph_->push_back(factor);
  }

  // Update latest pose
  latest_local_pose_ = current_estimate;
  latest_local_symbol_ = symbol;
}

void PoseGraphManager::inter_robot_loop_closure_callback(const cslam_loop_detection_interfaces::msg::
  InterRobotLoopClosure::ConstSharedPtr msg) {
    if (msg->success){
      gtsam::Pose3 measurement;
      transform_msg_to_pose3(msg->transform, measurement);

      gtsam::LabeledSymbol symbol_from(GRAPH_LABEL, ROBOT_LABEL(msg->robot0_id), msg->robot0_image_id);
      gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, ROBOT_LABEL(msg->robot1_id), msg->robot1_image_id);

      gtsam::BetweenFactor<gtsam::Pose3> factor(symbol_from, symbol_to, measurement, default_noise_model_);
      pose_graph_->push_back(factor);
    }
  }