#include "cslam/back_end/pose_graph_manager.h"

using namespace cslam;

PoseGraphManager::PoseGraphManager(std::shared_ptr<rclcpp::Node> &node)
    : node_(node), max_waiting_time_sec_(60, 0) {

  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);
  node_->get_parameter("pose_graph_manager_process_period_ms",
                       pose_graph_manager_process_period_ms_);
  node_->get_parameter("pose_graph_optimization_loop_period_ms",
                       pose_graph_optimization_loop_period_ms_);
  node_->get_parameter("heartbeat_period_sec", heartbeat_period_sec_);

  int max_waiting_param;
  node_->get_parameter("max_waiting_time_sec", max_waiting_param);
  max_waiting_time_sec_ = rclcpp::Duration(max_waiting_param, 0);

  odometry_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>(
          "keyframe_odom", 100,
          std::bind(&PoseGraphManager::odometry_callback, this,
                    std::placeholders::_1));

  inter_robot_loop_closure_subscriber_ = node->create_subscription<
      cslam_loop_detection_interfaces::msg::InterRobotLoopClosure>(
      "/inter_robot_loop_closure", 100,
      std::bind(&PoseGraphManager::inter_robot_loop_closure_callback, this,
                std::placeholders::_1));

  print_current_estimates_subscriber_ =
      node->create_subscription<std_msgs::msg::String>(
          "print_current_estimates", 100,
          std::bind(&PoseGraphManager::print_current_estimates_callback, this,
                    std::placeholders::_1));

  rotation_default_noise_std_ = 0.01;
  translation_default_noise_std_ = 0.1;
  Eigen::VectorXd sigmas(6);
  sigmas << rotation_default_noise_std_, rotation_default_noise_std_,
      rotation_default_noise_std_, translation_default_noise_std_,
      translation_default_noise_std_, translation_default_noise_std_;
  default_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  pose_graph_ = boost::make_shared<gtsam::NonlinearFactorGraph>();
  current_pose_estimates_ = boost::make_shared<gtsam::Values>();

  // Optimization timers
  optimization_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_manager_process_period_ms_),
      std::bind(&PoseGraphManager::optimization_callback, this));

  optimization_loop_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_optimization_loop_period_ms_),
      std::bind(&PoseGraphManager::optimization_loop_callback, this));

  // Publishers for optimization result
  debug_optimization_result_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizationResult>(
          "debug_optimization_result", 100);

  for (unsigned int i = 0; i < nb_robots_; i++) {
    optimized_estimates_publishers_.insert(
        {i, node->create_publisher<
                cslam_common_interfaces::msg::OptimizationResult>(
                "/r" + std::to_string(i) + "/optimized_estimates", 100)});
    reference_frame_per_robot_.insert(
        {i, geometry_msgs::msg::TransformStamped()});
  }

  optimized_estimates_subscriber_ = node->create_subscription<
      cslam_common_interfaces::msg::OptimizationResult>(
      "optimized_estimates", 100,
      std::bind(&PoseGraphManager::optimized_estimates_callback, this,
                std::placeholders::_1));

  optimizer_state_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizerState>(
          "optimizer_state", 100);

  // Initialize inter-robot loop closures measurements
  for (unsigned int i = 0; i < nb_robots_; i++) {
    for (unsigned int j = i + 1; j < nb_robots_; j++) {
      inter_robot_loop_closures_.insert(
          {{i, j}, std::vector<gtsam::BetweenFactor<gtsam::Pose3>>()});
    }
  }

  // Get neighbors ROS 2 objects
  get_current_neighbors_publisher_ =
      node->create_publisher<std_msgs::msg::String>("get_current_neighbors",
                                                    100);

  current_neighbors_subscriber_ = node->create_subscription<
      cslam_common_interfaces::msg::RobotIdsAndOrigin>(
      "current_neighbors", 100,
      std::bind(&PoseGraphManager::current_neighbors_callback, this,
                std::placeholders::_1));

  // PoseGraph ROS 2 objects
  for (unsigned int i = 0; i < nb_robots_; i++) {
    get_pose_graph_publishers_.insert(
        {i, node->create_publisher<cslam_common_interfaces::msg::RobotIds>(
                "/r" + std::to_string(i) + "/get_pose_graph", 100)});
    received_pose_graphs_.insert({i, false});
  }

  get_pose_graph_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::RobotIds>(
          "get_pose_graph", 100,
          std::bind(&PoseGraphManager::get_pose_graph_callback, this,
                    std::placeholders::_1));

  pose_graph_publisher_ =
      node->create_publisher<cslam_common_interfaces::msg::PoseGraph>(
          "/pose_graph", 100);

  pose_graph_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::PoseGraph>(
          "/pose_graph", 100,
          std::bind(&PoseGraphManager::pose_graph_callback, this,
                    std::placeholders::_1));

  // Optimizer
  optimizer_state_ = OptimizerState::IDLE;
  is_waiting_ = false;

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  tf_broadcaster_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_optimization_loop_period_ms_),
      std::bind(&PoseGraphManager::broadcast_tf_callback, this));

  heartbeat_publisher_ =
      node_->create_publisher<std_msgs::msg::UInt32>("heartbeat", 10);
  heartbeat_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds((unsigned int)heartbeat_period_sec_ * 1000),
      std::bind(&PoseGraphManager::heartbeat_timer_callback, this));

  reference_frame_per_robot_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::ReferenceFrames>(
          "reference_frames", rclcpp::QoS(1).transient_local());

  origin_robot_id_ = robot_id_;

  RCLCPP_INFO(node_->get_logger(), "Initialization done.");
}

void PoseGraphManager::reinitialize_received_pose_graphs() {
  for (unsigned int i = 0; i < nb_robots_; i++) {
    received_pose_graphs_[i] = false;
  }
  other_robots_graph_and_estimates_.clear();
  received_pose_graphs_connectivity_.clear();
  RCLCPP_INFO(node_->get_logger(), " CLEAR "); // TODO: remove
}

bool PoseGraphManager::check_received_pose_graphs() {
  bool received_all = true;
  for (auto id : current_neighbors_ids_.robots.ids) {
    received_all &= received_pose_graphs_[id];
  }
  return received_all;
}

void PoseGraphManager::odometry_callback(
    const cslam_common_interfaces::msg::KeyframeOdom::ConstSharedPtr msg) {

  gtsam::Pose3 current_estimate = odometry_msg_to_pose3(msg->odom);
  gtsam::LabeledSymbol symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), msg->id);

  current_pose_estimates_->insert(symbol, current_estimate);

  if (latest_local_symbol_ != gtsam::LabeledSymbol()) {
    gtsam::Pose3 odom_diff = current_estimate * latest_local_pose_.inverse();
    gtsam::BetweenFactor<gtsam::Pose3> factor(latest_local_symbol_, symbol,
                                              odom_diff, default_noise_model_);
    pose_graph_->push_back(factor);
  }

  // Update latest pose
  latest_local_pose_ = current_estimate;
  latest_local_symbol_ = symbol;
}

void PoseGraphManager::inter_robot_loop_closure_callback(
    const cslam_loop_detection_interfaces::msg::InterRobotLoopClosure::
        ConstSharedPtr msg) {
  if (msg->success) {
    gtsam::Pose3 measurement = transform_msg_to_pose3(msg->transform);

    unsigned char robot0_c = ROBOT_LABEL(msg->robot0_id);
    gtsam::LabeledSymbol symbol_from(GRAPH_LABEL, robot0_c,
                                     msg->robot0_image_id);
    unsigned char robot1_c = ROBOT_LABEL(msg->robot1_id);
    gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, robot1_c, msg->robot1_image_id);

    gtsam::BetweenFactor<gtsam::Pose3> factor =
        gtsam::BetweenFactor<gtsam::Pose3>(symbol_from, symbol_to, measurement,
                                           default_noise_model_);

    inter_robot_loop_closures_[{std::min(msg->robot0_id, msg->robot1_id),
                                std::max(msg->robot0_id, msg->robot1_id)}]
        .push_back(factor);
    if (msg->robot0_id == robot_id_){
      connected_robots_.insert(msg->robot1_id);
    } else if (msg->robot1_id == robot_id_) {
      connected_robots_.insert(msg->robot0_id);
    }
  }
}

void PoseGraphManager::print_current_estimates_callback(
    const std_msgs::msg::String::ConstSharedPtr msg) {
  gtsam::writeG2o(*pose_graph_, *current_pose_estimates_, msg->data);
}

void PoseGraphManager::current_neighbors_callback(
    const cslam_common_interfaces::msg::RobotIdsAndOrigin::ConstSharedPtr msg) {
  current_neighbors_ids_ = *msg;
  end_waiting();
  if (is_optimizer()) {
    optimizer_state_ = OptimizerState::POSEGRAPH_COLLECTION;
  }
}

bool PoseGraphManager::is_optimizer() {
  // Here we could implement a different priority check
  bool is_optimizer = true;
  for (unsigned int i = 0; i < current_neighbors_ids_.origins.ids.size(); i++) {
    if (origin_robot_id_ > current_neighbors_ids_.origins.ids[i]) {
      is_optimizer &= false;
    } else if (origin_robot_id_ == current_neighbors_ids_.origins.ids[i] &&
               robot_id_ > current_neighbors_ids_.robots.ids[i]) {
      is_optimizer &= false;
    }
  }
  if (current_neighbors_ids_.robots.ids.size() == 0 || current_pose_estimates_->size() == 0){
    is_optimizer = false;
  }
  return is_optimizer;
}

void PoseGraphManager::get_pose_graph_callback(
    const cslam_common_interfaces::msg::RobotIds::ConstSharedPtr msg) {
  cslam_common_interfaces::msg::PoseGraph out_msg;
  out_msg.robot_id = robot_id_;
  out_msg.values = gtsam_values_to_msg(current_pose_estimates_);
  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  graph->push_back(pose_graph_->begin(), pose_graph_->end());

  std::set<unsigned int> connected_robots;

  for (unsigned int i = 0; i < msg->ids.size(); i++) {
    for (unsigned int j = i + 1; j < msg->ids.size(); j++) {
      unsigned int min_robot_id = std::min(msg->ids[i], msg->ids[j]);
      unsigned int max_robot_id = std::max(msg->ids[i], msg->ids[j]);
      if (inter_robot_loop_closures_[{min_robot_id, max_robot_id}].size() > 0) {
        connected_robots.insert(max_robot_id); // TODO: double check this
        if (min_robot_id == robot_id_) {
          graph->push_back(
              inter_robot_loop_closures_[{min_robot_id, max_robot_id}].begin(),
              inter_robot_loop_closures_[{min_robot_id, max_robot_id}].end());
        }
      }
    }
  }

  out_msg.edges = gtsam_factors_to_msg(graph);
  RCLCPP_INFO(node_->get_logger(), " Pose graph publish robot " + std::to_string(robot_id_)); // TODO: remove
  for (auto id : connected_robots) {
    RCLCPP_INFO(node_->get_logger(), " Connected ids: " + std::to_string(id)); // TODO: remove
    out_msg.connected_robots.ids.push_back(id);
  }
  pose_graph_publisher_->publish(out_msg);
}

void PoseGraphManager::pose_graph_callback(
    const cslam_common_interfaces::msg::PoseGraph::ConstSharedPtr msg) {
  other_robots_graph_and_estimates_.insert(
      {msg->robot_id,
       {edges_msg_to_gtsam(msg->edges), values_msg_to_gtsam(msg->values)}});
  received_pose_graphs_[msg->robot_id] = true;
  received_pose_graphs_connectivity_.insert(
      {msg->robot_id, msg->connected_robots.ids});
  RCLCPP_INFO(node_->get_logger(), " Pose graph callback: " + std::to_string(msg->robot_id) + " < " + std::to_string(msg->connected_robots.ids.size())); // TODO: remove
  for (auto id : received_pose_graphs_connectivity_) { // TODO: remove
    RCLCPP_INFO(node_->get_logger(), " <<: " + std::to_string(id.first) + " < " + std::to_string(id.second.size())); // TODO: remove
    for (auto j : id.second) { // TODO: remove
      RCLCPP_INFO(node_->get_logger(), std::to_string(j)); // TODO: remove. should be 0?
    }
  }
  RCLCPP_INFO(node_->get_logger(), " ;;;;;;;;;;;;;;;;;;; "); // TODO: remove
  if (check_received_pose_graphs()) {
    end_waiting();
    optimizer_state_ = OptimizerState::OPTIMIZATION;
  }
  RCLCPP_INFO(node_->get_logger(), " ;;;;;;;;2222222;;;;; "); // TODO: remove
}

std::map<unsigned int, bool> PoseGraphManager::connected_robot_pose_graph() {
  RCLCPP_INFO(node_->get_logger(), " ;;Connected;; "); // TODO: remove
  RCLCPP_INFO(node_->get_logger(), " ;--- " + std::to_string(connected_robots_.size())); // TODO: remove
  if (connected_robots_.size() > 0){
    std::vector<unsigned int> v(connected_robots_.begin(), connected_robots_.end());
    RCLCPP_INFO(node_->get_logger(), " ;;Connected22;; "); // TODO: remove
    received_pose_graphs_connectivity_.insert({robot_id_, v});      
    RCLCPP_INFO(node_->get_logger(), " Current ids: " + std::to_string(robot_id_) + " " + std::to_string(v[0])); // TODO: remove
  }
  
  std::map<unsigned int, bool> is_robot_connected;
  is_robot_connected.insert({robot_id_, true});
  for (auto id : current_neighbors_ids_.robots.ids) {
    is_robot_connected.insert({id, false});
  }

  // Breadth First Search
  bool *visited = new bool[current_neighbors_ids_.robots.ids.size()];
  for (unsigned int i = 0; i < current_neighbors_ids_.robots.ids.size(); i++)
    visited[i] = false;

  std::list<unsigned int> queue;

  unsigned int current_id = robot_id_;
  visited[current_id] = true;
  queue.push_back(current_id);
  RCLCPP_INFO(node_->get_logger(), " ---------- "); // TODO: remove


  while (!queue.empty()) {
    current_id = queue.front();
    queue.pop_front();
    RCLCPP_INFO(node_->get_logger(), "!! = " + std::to_string(current_id) + " | " + std::to_string(received_pose_graphs_connectivity_[current_id].size())); // TODO: remove

    for (auto id : received_pose_graphs_connectivity_[current_id]) {
      is_robot_connected[id] = true;
      RCLCPP_INFO(node_->get_logger(), "** = " + std::to_string(current_id) + " | " + std::to_string(received_pose_graphs_connectivity_[current_id].size()) + " // " + std::to_string(id) + " ++ " + std::to_string(is_robot_connected[id]) + " $$ " + std::to_string(visited[id])); // TODO: remove
      
      if (!visited[id]) {
        visited[id] = true;
        queue.push_back(id);
      }
    }
  }
  for (auto id : received_pose_graphs_connectivity_) { // TODO: remove
    RCLCPP_INFO(node_->get_logger(), " <<: " + std::to_string(id.first) + " < " + std::to_string(id.second.size())); // TODO: remove
  }
  for (auto id : current_neighbors_ids_.robots.ids) { // TODO: remove
    RCLCPP_INFO(node_->get_logger(), " &&: " + std::to_string(id) + "  " + std::to_string(is_robot_connected[id])); // TODO: remove
  }
  RCLCPP_INFO(node_->get_logger(), " ============= "); // TODO: remove
  return is_robot_connected;
}

void PoseGraphManager::resquest_current_neighbors() {
  get_current_neighbors_publisher_->publish(std_msgs::msg::String());
}

void PoseGraphManager::start_waiting() {
  optimizer_state_ = OptimizerState::WAITING;
  is_waiting_ = true;
  start_waiting_time_ = node_->now();
}

void PoseGraphManager::end_waiting() { is_waiting_ = false; }

bool PoseGraphManager::check_waiting_timeout() {
  if ((node_->now() - start_waiting_time_) > max_waiting_time_sec_) {
    end_waiting();
    optimizer_state_ = OptimizerState::IDLE;
  }
  return is_waiting_;
}

void PoseGraphManager::optimization_callback() {
  if (optimizer_state_ == OptimizerState::IDLE &&
      current_pose_estimates_->size() > 0) {
    reinitialize_received_pose_graphs();
    resquest_current_neighbors();
    start_waiting();
  }
}

std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>
PoseGraphManager::aggregate_pose_graphs() {
  // Check connectivity
  RCLCPP_INFO(node_->get_logger(), " ;;;agg 0 "); // TODO: remove
  auto is_pose_graph_connected = connected_robot_pose_graph();
  RCLCPP_INFO(node_->get_logger(), " ;;;agg 1 "); // TODO: remove
  // Aggregate graphs
  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  auto estimates = boost::make_shared<gtsam::Values>();
  // Local graph
  graph->push_back(pose_graph_->begin(), pose_graph_->end());
  RCLCPP_INFO(node_->get_logger(), " ;;;agg 2 "); // TODO: remove
  auto included_robots_ids = current_neighbors_ids_;
  included_robots_ids.robots.ids.push_back(robot_id_);
  RCLCPP_INFO(node_->get_logger(), "included_robots : ");
  for (unsigned int i = 0; i < included_robots_ids.robots.ids.size(); i++) {
    RCLCPP_INFO(node_->get_logger(), std::to_string(included_robots_ids.robots.ids[i]) + " " + std::to_string(is_pose_graph_connected[included_robots_ids.robots.ids[i]])); // TODO: remove
    for (unsigned int j = i + 1; j < included_robots_ids.robots.ids.size();
         j++) {
      if (is_pose_graph_connected[included_robots_ids.robots.ids[i]] &&
          is_pose_graph_connected[included_robots_ids.robots.ids[j]]) {
        unsigned int min_id = std::min(included_robots_ids.robots.ids[i],
                                       included_robots_ids.robots.ids[j]);
        unsigned int max_id = std::max(included_robots_ids.robots.ids[i],
                                       included_robots_ids.robots.ids[j]);
        for (const auto &factor :
             inter_robot_loop_closures_[{min_id, max_id}]) {
          graph->push_back(factor);
        }
        RCLCPP_INFO(node_->get_logger(), "Inter robot : " + std::to_string(min_id) + " " + std::to_string(max_id) + " " + std::to_string(inter_robot_loop_closures_[{min_id, max_id}].size()));
      }
    }
  }
  estimates->insert(*current_pose_estimates_);
  // Add other robots graphs
  for (auto id : current_neighbors_ids_.robots.ids) {
    if (is_pose_graph_connected[id]) {
      RCLCPP_INFO(node_->get_logger(), "Poses : " + std::to_string(id) + " " + std::to_string(other_robots_graph_and_estimates_[id].second->size()));
      estimates->insert(*other_robots_graph_and_estimates_[id].second);
    }
  }
  for (auto id : current_neighbors_ids_.robots.ids) {
    RCLCPP_INFO(node_->get_logger(), "Other graph : " + std::to_string(id) + " " + std::to_string(other_robots_graph_and_estimates_[id].first->size()));
     
    for (const auto &factor_ : *other_robots_graph_and_estimates_[id].first) {
      auto factor =
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
              factor_);
      unsigned int robot0_id =
          ROBOT_ID(gtsam::LabeledSymbol(factor->key1()).label());
      unsigned int robot1_id =
          ROBOT_ID(gtsam::LabeledSymbol(factor->key2()).label());
      if (is_pose_graph_connected[robot0_id] &&
          is_pose_graph_connected[robot1_id]) {
        graph->push_back(factor);
      }
    }
  }
  return {graph, estimates};
}

void PoseGraphManager::optimized_estimates_callback(
    const cslam_common_interfaces::msg::OptimizationResult::ConstSharedPtr
        msg) {
  if (current_pose_estimates_->size() > 0) {
    auto optimized_estimates = values_msg_to_gtsam(msg->estimates);
    current_pose_estimates_->update(*optimized_estimates);
    origin_robot_id_ = msg->origin_robot_id;
    gtsam::LabeledSymbol first_symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), 0);

    gtsam::Pose3 first_pose;
    if (current_pose_estimates_->exists(first_symbol))
    {
      first_pose = current_pose_estimates_->at<gtsam::Pose3>(first_symbol);
    }
    update_transform_to_origin(first_pose);
  }
}

void PoseGraphManager::share_optimized_estimates(
    const gtsam::Values &estimates) {
  auto included_robots_ids = current_neighbors_ids_;
  included_robots_ids.robots.ids.push_back(robot_id_);
  for (unsigned int i = 0; i < included_robots_ids.robots.ids.size(); i++) {
    cslam_common_interfaces::msg::OptimizationResult msg;
    msg.success = true;
    msg.origin_robot_id = origin_robot_id_;
    msg.estimates =
        gtsam_values_to_msg(estimates.filter(gtsam::LabeledSymbol::LabelTest(
            ROBOT_LABEL(included_robots_ids.robots.ids[i]))));
    optimized_estimates_publishers_[included_robots_ids.robots.ids[i]]->publish(
        msg);
  }
}

void PoseGraphManager::heartbeat_timer_callback() {
  std_msgs::msg::UInt32 msg;
  msg.data = origin_robot_id_;
  heartbeat_publisher_->publish(msg);
}

void PoseGraphManager::update_transform_to_origin(const gtsam::Pose3 &pose) {
  rclcpp::Time now = node_->get_clock()->now();
  origin_to_first_pose_.header.stamp = now;
  origin_to_first_pose_.header.frame_id =
      "robot_" + std::to_string(origin_robot_id_);
  origin_to_first_pose_.child_frame_id = "robot_" + std::to_string(robot_id_);

  origin_to_first_pose_.transform = gtsam_pose_to_transform_msg(pose);

  // Update the reference frame
  // This is the key info for many tasks since it allows conversions from
  // one robot reference frame to another.
  for (auto i : current_neighbors_ids_.robots.ids) {
    reference_frame_per_robot_[i] = origin_to_first_pose_;
  }
  cslam_common_interfaces::msg::ReferenceFrames msg;
  for (const auto &ref : reference_frame_per_robot_) {
    msg.robots.ids.push_back(ref.first);
    msg.reference_frames.push_back(ref.second);
  }
  reference_frame_per_robot_publisher_->publish(msg);
}

void PoseGraphManager::broadcast_tf_callback() {
  rclcpp::Time now = node_->get_clock()->now();
  origin_to_first_pose_.header.stamp = now;
  // Useful for visualization.
  // For tasks purposes use reference_frame_per_robot_ instead
  if (origin_to_first_pose_.header.frame_id !=
      origin_to_first_pose_.child_frame_id) {
    tf_broadcaster_->sendTransform(origin_to_first_pose_);
  }
}

void PoseGraphManager::perform_optimization() {

  // Build global pose graph
  RCLCPP_INFO(node_->get_logger(), "Perf opt 0  ");
  gtsam::writeG2o(*pose_graph_, *current_pose_estimates_, "before_aggregation.g2o");// TODO: remove
  RCLCPP_INFO(node_->get_logger(), " ;;;;Perf oopt "); // TODO: remove
  auto graph_and_estimates = aggregate_pose_graphs();

  RCLCPP_INFO(node_->get_logger(), "Perf opt 1  ");
  // Add prior
  gtsam::LabeledSymbol first_symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), 0);
  if (!current_pose_estimates_->exists(first_symbol)){
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Perf opt 2  ");
  graph_and_estimates.first->addPrior(
      first_symbol, current_pose_estimates_->at<gtsam::Pose3>(first_symbol),
      default_noise_model_);

  RCLCPP_INFO(node_->get_logger(), "Perf opt 3  ");
  gtsam::writeG2o(*graph_and_estimates.first, *graph_and_estimates.second, "before_optimization.g2o");// TODO: remove

  RCLCPP_INFO(node_->get_logger(), "Perf opt 3.1  ");
  // // Optimize graph
  gtsam::GncParams<gtsam::LevenbergMarquardtParams> params;
  gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
      optimizer(*graph_and_estimates.first, *graph_and_estimates.second,
                params);
  RCLCPP_INFO(node_->get_logger(), "Perf opt 3.2  ");
  gtsam::Values result = optimizer.optimize();
  RCLCPP_INFO(node_->get_logger(), "Perf opt 4  ");

  gtsam::writeG2o(*pose_graph_, result, "after_optimization.g2o");// TODO: remove

  // Share results
  // share_optimized_estimates(result); // TODO: reestablish

  RCLCPP_INFO(node_->get_logger(), "Perf opt 5  "); // TODO: remove
  // // Publish result info for monitoring
  cslam_common_interfaces::msg::OptimizationResult msg;
  msg.success = true;
  msg.factors = gtsam_factors_to_msg(
      graph_and_estimates.first); // TODO: Do not fill, unless debugging mode
  msg.estimates =
      gtsam_values_to_msg(result); // TODO: Do not fill, unless debugging mode
  debug_optimization_result_publisher_->publish(
      msg); // TODO: publish on debug mode
  RCLCPP_INFO(node_->get_logger(), "Perf opt 6  "); // TODO: remove
}

void PoseGraphManager::optimization_loop_callback() {
  if (!current_pose_estimates_->empty()) {
    if (optimizer_state_ ==
        OptimizerState::POSEGRAPH_COLLECTION) // TODO: Document
    {
      if (current_neighbors_ids_.robots.ids.size() > 0) {
        for (auto id : current_neighbors_ids_.robots.ids) {
          auto current_robots_ids = current_neighbors_ids_;
          current_robots_ids.robots.ids.push_back(robot_id_);
          get_pose_graph_publishers_[id]->publish(current_robots_ids.robots);
        }
        start_waiting();
      } else {
        optimizer_state_ = OptimizerState::IDLE;
      }
    } else if (optimizer_state_ == OptimizerState::OPTIMIZATION) {
      // Call optimization
      perform_optimization();
      optimizer_state_ = OptimizerState::IDLE;
    } else if (optimizer_state_ == OptimizerState::WAITING) {
      check_waiting_timeout();
    }
  }
  cslam_common_interfaces::msg::OptimizerState state_msg;
  state_msg.state = optimizer_state_;
  optimizer_state_publisher_->publish(state_msg);
}