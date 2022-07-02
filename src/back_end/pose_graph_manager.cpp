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
        "/inter_robot_loop_closure", 100,
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

  // Optimization timer
  optimization_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(pose_graph_manager_process_period_ms_), std::bind(&PoseGraphManager::optimization_callback, this));

  // Publisher for optimization result
  optimization_result_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizationResult>(
          "optimization_result", 100);

  // Initialize inter-robot loop closures measurements
  for (unsigned int i = 0; i < nb_robots_; i++)
  {
    for (unsigned int j = i + 1; j < nb_robots_; j++)
    {
      inter_robot_loop_closures_.insert(std::pair{std::pair{i, j}, std::vector<gtsam::BetweenFactor<gtsam::Pose3>>()});
    }
  }
  
  // Add prior 
  // TODO: not for decentralized
  gtsam::LabeledSymbol first_symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), 0);
  pose_graph_->addPrior(first_symbol, gtsam::Pose3(), default_noise_model_);
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

      unsigned char robot0_c = ROBOT_LABEL(msg->robot0_id);
      gtsam::LabeledSymbol symbol_from(GRAPH_LABEL, robot0_c, msg->robot0_image_id);
      unsigned char robot1_c = ROBOT_LABEL(msg->robot1_id);
      gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, robot1_c, msg->robot1_image_id);

      gtsam::BetweenFactor<gtsam::Pose3> factor(symbol_from, symbol_to, measurement, default_noise_model_);
      
      inter_robot_loop_closures_[std::min(robot0_c, robot1_c), std::max(robot0_c, robot1_c)].emplace_back(factor);
    }
  }

void PoseGraphManager::optimization_callback(){
  if (!current_pose_estimates_->empty())
  {
    // TODO: Ask for pose graph
    // TODO: Compute graph
    /*gtsam::GncParams<gtsam::LevenbergMarquardtParams> params;
    gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> optimizer(*pose_graph_, *current_pose_estimates_, params);
    gtsam::Values result = optimizer.optimize();

    // TODO: print result
    // TODO: publish a TF

    // Publish result info for monitoring
    cslam_common_interfaces::msg::OptimizationResult msg;
    msg.success = true;
    optimization_result_publisher_->publish(msg);*/
  }
}