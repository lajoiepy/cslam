#include "cslam/back_end/decentralized_pgo.h"

#define MAP_FRAME_ID(id) "robot" + std::to_string(id) + "_map"
#define CURRENT_FRAME_ID(id) "robot" + std::to_string(id) + "_current_pose"
#define LATEST_OPTIMIZED_FRAME_ID(id) "robot" + std::to_string(id) + "_latest_optimized_pose"

using namespace cslam;

DecentralizedPGO::DecentralizedPGO(std::shared_ptr<rclcpp::Node> &node)
    : node_(node), max_waiting_time_sec_(60, 0)
{

  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);
  node_->get_parameter("backend.pose_graph_optimization_start_period_ms",
                       pose_graph_optimization_start_period_ms_);
  node_->get_parameter("backend.pose_graph_optimization_loop_period_ms",
                       pose_graph_optimization_loop_period_ms_);
  node_->get_parameter("neighbor_management.heartbeat_period_sec", heartbeat_period_sec_);
  node->get_parameter("backend.enable_log_optimization_files",
                      enable_log_optimization_files_);
  node->get_parameter("backend.log_optimization_files_path",
                      log_optimization_files_path_);
  node_->get_parameter("backend.visualization_period_ms",
                       visualization_period_ms_);
  node_->get_parameter("backend.broadcast_tf_frames",
                       broadcast_tf_frames_);

  int max_waiting_param;
  node_->get_parameter("backend.max_waiting_time_sec", max_waiting_param);
  max_waiting_time_sec_ = rclcpp::Duration(max_waiting_param, 0);

  odometry_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>(
          "keyframe_odom", 1000,
          std::bind(&DecentralizedPGO::odometry_callback, this,
                    std::placeholders::_1));

  intra_robot_loop_closure_subscriber_ = node->create_subscription<
      cslam_loop_detection_interfaces::msg::IntraRobotLoopClosure>(
      "intra_robot_loop_closure", 1000,
      std::bind(&DecentralizedPGO::intra_robot_loop_closure_callback, this,
                std::placeholders::_1));

  inter_robot_loop_closure_subscriber_ = node->create_subscription<
      cslam_loop_detection_interfaces::msg::InterRobotLoopClosure>(
      "/inter_robot_loop_closure", 1000,
      std::bind(&DecentralizedPGO::inter_robot_loop_closure_callback, this,
                std::placeholders::_1));

  write_current_estimates_subscriber_ =
      node->create_subscription<std_msgs::msg::String>(
          "print_current_estimates", 100,
          std::bind(&DecentralizedPGO::write_current_estimates_callback, this,
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
  odometry_pose_estimates_ = boost::make_shared<gtsam::Values>();

  // Optimization timers
  optimization_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_optimization_start_period_ms_),
      std::bind(&DecentralizedPGO::optimization_callback, this));

  optimization_loop_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_optimization_loop_period_ms_),
      std::bind(&DecentralizedPGO::optimization_loop_callback, this));

  if (visualization_period_ms_ > 1e-6)
  {
    RCLCPP_INFO(node_->get_logger(), "Visualization enabled.");
    visualization_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(visualization_period_ms_),
        std::bind(&DecentralizedPGO::visualization_callback, this));
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Visualization disabled.");
  }

  // Publishers for optimization result
  debug_optimization_result_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizationResult>(
          "debug_optimization_result", 100);

  for (unsigned int i = 0; i < nb_robots_; i++)
  {
    optimized_estimates_publishers_.insert(
        {i, node->create_publisher<
                cslam_common_interfaces::msg::OptimizationResult>(
                "/r" + std::to_string(i) + "/optimized_estimates", 100)});
  }

  optimized_estimates_subscriber_ = node->create_subscription<
      cslam_common_interfaces::msg::OptimizationResult>(
      "optimized_estimates", 100,
      std::bind(&DecentralizedPGO::optimized_estimates_callback, this,
                std::placeholders::_1));

  optimizer_state_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizerState>(
          "optimizer_state", 100);

  // Initialize inter-robot loop closures measurements
  for (unsigned int i = 0; i < nb_robots_; i++)
  {
    for (unsigned int j = i + 1; j < nb_robots_; j++)
    {
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
      std::bind(&DecentralizedPGO::current_neighbors_callback, this,
                std::placeholders::_1));

  // PoseGraph ROS 2 objects
  for (unsigned int i = 0; i < nb_robots_; i++)
  {
    get_pose_graph_publishers_.insert(
        {i, node->create_publisher<cslam_common_interfaces::msg::RobotIds>(
                "/r" + std::to_string(i) + "/get_pose_graph", 100)});
    received_pose_graphs_.insert({i, false});
  }

  get_pose_graph_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::RobotIds>(
          "get_pose_graph", 100,
          std::bind(&DecentralizedPGO::get_pose_graph_callback, this,
                    std::placeholders::_1));

  pose_graph_publisher_ =
      node->create_publisher<cslam_common_interfaces::msg::PoseGraph>(
          "/pose_graph", 100);

  pose_graph_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::PoseGraph>(
          "/pose_graph", 100,
          std::bind(&DecentralizedPGO::pose_graph_callback, this,
                    std::placeholders::_1));

  visualization_pose_graph_publisher_ =
      node->create_publisher<cslam_common_interfaces::msg::PoseGraph>(
          "/viz/pose_graph", 100);

  // Optimizer
  optimizer_state_ = OptimizerState::IDLE;
  is_waiting_ = false;
  optimization_count_ = 0;

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  if (broadcast_tf_frames_)
  {
    tf_broadcaster_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(pose_graph_optimization_loop_period_ms_),
        std::bind(&DecentralizedPGO::broadcast_tf_callback, this));
  }

  heartbeat_publisher_ =
      node_->create_publisher<std_msgs::msg::UInt32>("heartbeat", 10);
  heartbeat_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds((unsigned int)heartbeat_period_sec_ * 1000),
      std::bind(&DecentralizedPGO::heartbeat_timer_callback, this));

  reference_frame_per_robot_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::ReferenceFrames>(
          "reference_frames", rclcpp::QoS(1).transient_local());

  origin_robot_id_ = robot_id_;

  RCLCPP_INFO(node_->get_logger(), "Initialization done.");
}

void DecentralizedPGO::reinitialize_received_pose_graphs()
{
  for (unsigned int i = 0; i < nb_robots_; i++)
  {
    received_pose_graphs_[i] = false;
  }
  other_robots_graph_and_estimates_.clear();
  received_pose_graphs_connectivity_.clear();
}

bool DecentralizedPGO::check_received_pose_graphs()
{
  bool received_all = true;
  for (auto id : current_neighbors_ids_.robots.ids)
  {
    received_all &= received_pose_graphs_[id];
  }
  return received_all;
}

void DecentralizedPGO::odometry_callback(
    const cslam_common_interfaces::msg::KeyframeOdom::ConstSharedPtr msg)
{

  gtsam::Pose3 current_estimate = odometry_msg_to_pose3(msg->odom);
  gtsam::LabeledSymbol symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), msg->id);

  odometry_pose_estimates_->insert(symbol, current_estimate);
  if (msg->id == 0)
  {
    current_pose_estimates_->insert(symbol, current_estimate);
  }

  if (latest_local_symbol_ != gtsam::LabeledSymbol())
  {
    gtsam::Pose3 odom_diff = latest_local_pose_.inverse() * current_estimate;
    gtsam::BetweenFactor<gtsam::Pose3> factor(latest_local_symbol_, symbol,
                                              odom_diff, default_noise_model_);
    pose_graph_->push_back(factor);
  }

  // Update latest pose
  latest_local_pose_ = current_estimate;
  latest_local_symbol_ = symbol;
}

void DecentralizedPGO::intra_robot_loop_closure_callback(
    const cslam_loop_detection_interfaces::msg::IntraRobotLoopClosure::
        ConstSharedPtr msg)
{
  if (msg->success)
  {
    gtsam::Pose3 measurement = transform_msg_to_pose3(msg->transform);

    gtsam::LabeledSymbol symbol_from(GRAPH_LABEL, ROBOT_LABEL(robot_id_),
                                     msg->keyframe0_id);
    gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, ROBOT_LABEL(robot_id_),
                                   msg->keyframe1_id);

    gtsam::BetweenFactor<gtsam::Pose3> factor =
        gtsam::BetweenFactor<gtsam::Pose3>(symbol_from, symbol_to, measurement,
                                           default_noise_model_);

    pose_graph_->push_back(factor);
  }
}

void DecentralizedPGO::inter_robot_loop_closure_callback(
    const cslam_loop_detection_interfaces::msg::InterRobotLoopClosure::
        ConstSharedPtr msg)
{
  if (msg->success)
  {
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
    if (msg->robot0_id == robot_id_)
    {
      connected_robots_.insert(msg->robot1_id);
    }
    else if (msg->robot1_id == robot_id_)
    {
      connected_robots_.insert(msg->robot0_id);
    }
  }
}

void DecentralizedPGO::write_current_estimates_callback(
    const std_msgs::msg::String::ConstSharedPtr msg)
{
  try{
    gtsam::writeG2o(*pose_graph_, *current_pose_estimates_, msg->data);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Error while writing estimates: %s", e.what());
  }
}

void DecentralizedPGO::current_neighbors_callback(
    const cslam_common_interfaces::msg::RobotIdsAndOrigin::ConstSharedPtr msg)
{
  current_neighbors_ids_ = *msg;
  end_waiting();
  if (is_optimizer())
  {
    optimizer_state_ = OptimizerState::POSEGRAPH_COLLECTION;
  }
  else
  {
    optimizer_state_ = OptimizerState::IDLE;
  }
}

bool DecentralizedPGO::is_optimizer()
{
  // Here we could implement a different priority check
  bool is_optimizer = true;
  for (unsigned int i = 0; i < current_neighbors_ids_.origins.ids.size(); i++)
  {
    if (origin_robot_id_ > current_neighbors_ids_.origins.ids[i])
    {
      is_optimizer = false;
    }
    else if (origin_robot_id_ == current_neighbors_ids_.origins.ids[i] &&
             robot_id_ > current_neighbors_ids_.robots.ids[i])
    {
      is_optimizer = false;
    }
  }
  if (current_neighbors_ids_.robots.ids.size() == 0 ||
      odometry_pose_estimates_->size() == 0)
  {
    is_optimizer = false;
  }
  return is_optimizer;
}

void DecentralizedPGO::get_pose_graph_callback(
    const cslam_common_interfaces::msg::RobotIds::ConstSharedPtr msg)
{
  cslam_common_interfaces::msg::PoseGraph out_msg;
  out_msg.robot_id = robot_id_;
  out_msg.values = gtsam_values_to_msg(odometry_pose_estimates_);
  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  graph->push_back(pose_graph_->begin(), pose_graph_->end());

  std::set<unsigned int> connected_robots;

  for (unsigned int i = 0; i < msg->ids.size(); i++)
  {
    for (unsigned int j = i + 1; j < msg->ids.size(); j++)
    {
      unsigned int min_robot_id = std::min(msg->ids[i], msg->ids[j]);
      unsigned int max_robot_id = std::max(msg->ids[i], msg->ids[j]);
      if (inter_robot_loop_closures_[{min_robot_id, max_robot_id}].size() > 0 &&
          (min_robot_id == robot_id_ || max_robot_id == robot_id_))
      {
        connected_robots.insert(min_robot_id);
        connected_robots.insert(max_robot_id);
        if (min_robot_id == robot_id_)
        {
          graph->push_back(
              inter_robot_loop_closures_[{min_robot_id, max_robot_id}].begin(),
              inter_robot_loop_closures_[{min_robot_id, max_robot_id}].end());
        }
      }
    }
  }

  out_msg.edges = gtsam_factors_to_msg(graph);
  for (auto id : connected_robots)
  {
    if (id != robot_id_)
    {
      out_msg.connected_robots.ids.push_back(id);
    }
  }
  pose_graph_publisher_->publish(out_msg);
  tentative_local_pose_at_latest_optimization_ = latest_local_pose_;
}

void DecentralizedPGO::pose_graph_callback(
    const cslam_common_interfaces::msg::PoseGraph::ConstSharedPtr msg)
{
  if (optimizer_state_ == OptimizerState::WAITING_FOR_NEIGHBORS_POSEGRAPHS)
  {
    other_robots_graph_and_estimates_.insert(
        {msg->robot_id,
         {edges_msg_to_gtsam(msg->edges), values_msg_to_gtsam(msg->values)}});
    received_pose_graphs_[msg->robot_id] = true;
    received_pose_graphs_connectivity_.insert(
        {msg->robot_id, msg->connected_robots.ids});
    if (check_received_pose_graphs())
    {
      end_waiting();
      optimizer_state_ = OptimizerState::START_OPTIMIZATION;
    }
  }
}

std::map<unsigned int, bool> DecentralizedPGO::connected_robot_pose_graph()
{
  if (connected_robots_.size() > 0)
  {
    std::vector<unsigned int> v(connected_robots_.begin(),
                                connected_robots_.end());
    received_pose_graphs_connectivity_.insert({robot_id_, v});
  }

  std::map<unsigned int, bool> is_robot_connected;
  is_robot_connected.insert({robot_id_, true});
  for (auto id : current_neighbors_ids_.robots.ids)
  {
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

  while (!queue.empty())
  {
    current_id = queue.front();
    queue.pop_front();

    for (auto id : received_pose_graphs_connectivity_[current_id])
    {
      is_robot_connected[id] = true;

      if (!visited[id])
      {
        visited[id] = true;
        queue.push_back(id);
      }
    }
  }
  return is_robot_connected;
}

void DecentralizedPGO::resquest_current_neighbors()
{
  get_current_neighbors_publisher_->publish(std_msgs::msg::String());
}

void DecentralizedPGO::start_waiting()
{
  if (optimizer_state_ == OptimizerState::IDLE)
  {
    optimizer_state_ = OptimizerState::WAITING_FOR_NEIGHBORS_INFO;
  }
  else if (optimizer_state_ == OptimizerState::POSEGRAPH_COLLECTION)
  {
    optimizer_state_ = OptimizerState::WAITING_FOR_NEIGHBORS_POSEGRAPHS;
  }
  is_waiting_ = true;
  start_waiting_time_ = node_->now();
}

void DecentralizedPGO::end_waiting() { is_waiting_ = false; }

bool DecentralizedPGO::is_waiting() { return is_waiting_; }

bool DecentralizedPGO::check_waiting_timeout()
{
  if ((node_->now() - start_waiting_time_) > max_waiting_time_sec_)
  {
    end_waiting();
    optimizer_state_ = OptimizerState::IDLE;
    RCLCPP_INFO(node_->get_logger(), "Timeout: " + std::to_string(robot_id_));
  }
  return is_waiting();
}

void DecentralizedPGO::optimization_callback()
{
  if (optimizer_state_ == OptimizerState::IDLE &&
      odometry_pose_estimates_->size() > 0)
  {
    reinitialize_received_pose_graphs();
    resquest_current_neighbors();
    start_waiting();
  }
}

std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>
DecentralizedPGO::aggregate_pose_graphs()
{
  // Check connectivity
  auto is_pose_graph_connected = connected_robot_pose_graph();
  // Aggregate graphs
  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  auto estimates = boost::make_shared<gtsam::Values>();
  // Local graph
  graph->push_back(pose_graph_->begin(), pose_graph_->end());
  estimates->insert(*odometry_pose_estimates_);
  tentative_local_pose_at_latest_optimization_ = latest_local_pose_;
  // Add other robots graphs
  for (auto id : current_neighbors_ids_.robots.ids)
  {
    if (is_pose_graph_connected[id])
    {
      estimates->insert(*other_robots_graph_and_estimates_[id].second);
    }
  }
  // Add local inter-robot loop closures
  auto included_robots_ids = current_neighbors_ids_;
  included_robots_ids.robots.ids.push_back(robot_id_);
  for (unsigned int i = 0; i < included_robots_ids.robots.ids.size(); i++)
  {
    for (unsigned int j = i + 1; j < included_robots_ids.robots.ids.size();
         j++)
    {
      if (is_pose_graph_connected[included_robots_ids.robots.ids[i]] &&
          is_pose_graph_connected[included_robots_ids.robots.ids[j]])
      {
        unsigned int min_id = std::min(included_robots_ids.robots.ids[i],
                                       included_robots_ids.robots.ids[j]);
        unsigned int max_id = std::max(included_robots_ids.robots.ids[i],
                                       included_robots_ids.robots.ids[j]);
        for (const auto &factor :
             inter_robot_loop_closures_[{min_id, max_id}])
        {
          if (estimates->exists(factor.key1()) &&
              estimates->exists(factor.key2()))
          {
            graph->push_back(factor);
          }
        }
      }
    }
  }
  // Add other robots factors
  for (auto id : current_neighbors_ids_.robots.ids)
  {

    for (const auto &factor_ : *other_robots_graph_and_estimates_[id].first)
    {
      auto factor =
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
              factor_);
      unsigned int robot0_id =
          ROBOT_ID(gtsam::LabeledSymbol(factor->key1()).label());
      unsigned int robot1_id =
          ROBOT_ID(gtsam::LabeledSymbol(factor->key2()).label());
      if (is_pose_graph_connected[robot0_id] &&
          is_pose_graph_connected[robot1_id])
      {
        if (estimates->exists(factor->key1()) &&
            estimates->exists(factor->key2()))
        {
          graph->push_back(factor);
        }
      }
    }
  }
  return {graph, estimates};
}

void DecentralizedPGO::optimized_estimates_callback(
    const cslam_common_interfaces::msg::OptimizationResult::ConstSharedPtr
        msg)
{
  if (odometry_pose_estimates_->size() > 0 && msg->estimates.size() > 0)
  {
    current_pose_estimates_ = values_msg_to_gtsam(msg->estimates);
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

void DecentralizedPGO::share_optimized_estimates(
    const gtsam::Values &estimates)
{
  auto included_robots_ids = current_neighbors_ids_;
  included_robots_ids.robots.ids.push_back(robot_id_);
  for (unsigned int i = 0; i < included_robots_ids.robots.ids.size(); i++)
  {
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

void DecentralizedPGO::heartbeat_timer_callback()
{
  std_msgs::msg::UInt32 msg;
  msg.data = origin_robot_id_;
  heartbeat_publisher_->publish(msg);
}

void DecentralizedPGO::visualization_callback()
{
  if (visualization_pose_graph_publisher_->get_subscription_count() > 0)
  {
    cslam_common_interfaces::msg::PoseGraph out_msg;
    out_msg.robot_id = robot_id_;
    out_msg.origin_robot_id = origin_robot_id_;
    out_msg.values = gtsam_values_to_msg(current_pose_estimates_);
    auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
    graph->push_back(pose_graph_->begin(), pose_graph_->end());

    for (unsigned int i = 0; i < nb_robots_; i++)
    {
      for (unsigned int j = i + 1; j < nb_robots_; j++)
      {
        unsigned int min_robot_id = std::min(i, j);
        unsigned int max_robot_id = std::max(i, j);
        if (inter_robot_loop_closures_[{min_robot_id, max_robot_id}].size() > 0 &&
            (min_robot_id == robot_id_ || max_robot_id == robot_id_))
        {
          if (min_robot_id == robot_id_)
          {
            graph->push_back(
                inter_robot_loop_closures_[{min_robot_id, max_robot_id}].begin(),
                inter_robot_loop_closures_[{min_robot_id, max_robot_id}].end());
          }
        }
      }
    }

    out_msg.edges = gtsam_factors_to_msg(graph);
    visualization_pose_graph_publisher_->publish(out_msg);
  }
}

void DecentralizedPGO::update_transform_to_origin(const gtsam::Pose3 &pose)
{
  rclcpp::Time now = node_->get_clock()->now();
  origin_to_first_pose_.header.stamp = now;
  origin_to_first_pose_.header.frame_id = MAP_FRAME_ID(origin_robot_id_);
  origin_to_first_pose_.child_frame_id = MAP_FRAME_ID(robot_id_);

  origin_to_first_pose_.transform = gtsam_pose_to_transform_msg(pose);

  // Update the reference frame
  // This is the key info for many tasks since it allows conversions from
  // one robot reference frame to another.
  if (reference_frame_per_robot_publisher_->get_subscription_count() > 0)
  {
    cslam_common_interfaces::msg::ReferenceFrames msg;
    msg.robot_id = robot_id_;
    msg.origin_to_local = origin_to_first_pose_;
    reference_frame_per_robot_publisher_->publish(msg);
  }
  // Store for TF
  local_pose_at_latest_optimization_ = tentative_local_pose_at_latest_optimization_;
  latest_optimized_pose_ = current_pose_estimates_->at<gtsam::Pose3>(current_pose_estimates_->keys().back());
}

void DecentralizedPGO::broadcast_tf_callback()
{
  // Useful for visualization.
  // For tasks purposes you might want to use reference_frame_per_robot_ instead
  // Since it is updated only when a new optimization is performed.

  // origin to local map
  rclcpp::Time now = node_->get_clock()->now();
  origin_to_first_pose_.header.stamp = now;
  if (origin_to_first_pose_.header.frame_id !=
      origin_to_first_pose_.child_frame_id)
  {
    RCLCPP_INFO(node_->get_logger(), "Broadcasting tf from %s to %s",
                origin_to_first_pose_.header.frame_id.c_str(),
                origin_to_first_pose_.child_frame_id.c_str());
    tf_broadcaster_->sendTransform(origin_to_first_pose_);
  }

  // origin to latest optimized pose
  geometry_msgs::msg::TransformStamped latest_optimized_pose_msg;
  latest_optimized_pose_msg.header.stamp = now;
  latest_optimized_pose_msg.header.frame_id = MAP_FRAME_ID(origin_robot_id_);
  latest_optimized_pose_msg.child_frame_id = LATEST_OPTIMIZED_FRAME_ID(robot_id_);
  latest_optimized_pose_msg.transform = gtsam_pose_to_transform_msg(
        latest_optimized_pose_);
  tf_broadcaster_->sendTransform(latest_optimized_pose_msg);

  // latest optimized pose to latest local pose (odometry alone)
  geometry_msgs::msg::TransformStamped current_pose_msg;
  current_pose_msg.header.stamp = now;
  current_pose_msg.header.frame_id = LATEST_OPTIMIZED_FRAME_ID(robot_id_);
  current_pose_msg.child_frame_id = CURRENT_FRAME_ID(robot_id_);
  gtsam::Pose3 current_pose3 = local_pose_at_latest_optimization_.inverse() * latest_local_pose_;
  current_pose_msg.transform = gtsam_pose_to_transform_msg(current_pose3);
  tf_broadcaster_->sendTransform(current_pose_msg);
}

gtsam::Values
DecentralizedPGO::optimize(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                           const gtsam::Values::shared_ptr &initial)
{
  gtsam::Values result;
  try{
    gtsam::GncParams<gtsam::LevenbergMarquardtParams> params;
    gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
        optimizer(*graph, *initial, params);
    result = optimizer.optimize();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Optimization failed: %s", e.what());
    result = *initial;
  }
  return result;
}

void DecentralizedPGO::start_optimization()
{
  // Build global pose graph
  aggregate_pose_graph_ = aggregate_pose_graphs();

  if (enable_log_optimization_files_)
  {
    try{
      gtsam::writeG2o(*aggregate_pose_graph_.first, *aggregate_pose_graph_.second,
                      log_optimization_files_path_ + "/" +
                          std::to_string(optimization_count_) + "_robot" +
                          std::to_string(robot_id_) + "_before_optimization.g2o");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Error writing g2o file: %s", e.what());
    }
  }

  // Add prior
  // Use first pose of current estimate
  gtsam::LabeledSymbol first_symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), 0);

  if (!current_pose_estimates_->exists(first_symbol))
  {
    return;
  }

  aggregate_pose_graph_.first->addPrior(
      first_symbol, current_pose_estimates_->at<gtsam::Pose3>(first_symbol),
      default_noise_model_);

  // Optimize graph
  optimization_result_ =
      std::async(&DecentralizedPGO::optimize, this, aggregate_pose_graph_.first,
                 aggregate_pose_graph_.second);
  optimizer_state_ = OptimizerState::OPTIMIZATION;
}

void DecentralizedPGO::check_result_and_finish_optimization()
{
  auto status = optimization_result_.wait_for(std::chrono::milliseconds(0));

  if (status == std::future_status::ready)
  {
    RCLCPP_INFO(node_->get_logger(), "Pose Graph Optimization complete.");
    auto result = optimization_result_.get();
    optimization_count_++;

    // Share results
    share_optimized_estimates(result);
    optimizer_state_ = OptimizerState::IDLE;

    if (enable_log_optimization_files_)
    {
      try{
        gtsam::writeG2o(
            *aggregate_pose_graph_.first, result,
            log_optimization_files_path_ + "/" + std::to_string(optimization_count_) +
                "_robot" + std::to_string(robot_id_) + "_after_optimization.g2o");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error writing g2o file: %s", e.what());
      }
    }

    // Publish result info for monitoring
    if (debug_optimization_result_publisher_->get_subscription_count() > 0)
    {
      cslam_common_interfaces::msg::OptimizationResult msg;
      msg.success = true;
      msg.factors = gtsam_factors_to_msg(aggregate_pose_graph_.first);
      msg.estimates = gtsam_values_to_msg(result);
      debug_optimization_result_publisher_->publish(msg);
    }
  }
}

void DecentralizedPGO::optimization_loop_callback()
{
  if (!odometry_pose_estimates_->empty())
  {
    if (optimizer_state_ ==
        OptimizerState::POSEGRAPH_COLLECTION)
    {
      if (current_neighbors_ids_.robots.ids.size() > 0)
      {
        for (auto id : current_neighbors_ids_.robots.ids)
        {
          auto current_robots_ids = current_neighbors_ids_;
          current_robots_ids.robots.ids.push_back(robot_id_);
          get_pose_graph_publishers_[id]->publish(current_robots_ids.robots);
        }
        start_waiting();
      }
      else
      {
        optimizer_state_ = OptimizerState::IDLE;
      }
    }
    else if (optimizer_state_ == OptimizerState::START_OPTIMIZATION)
    {
      // Call optimization
      start_optimization();
    }
    else if (optimizer_state_ == OptimizerState::OPTIMIZATION)
    {
      check_result_and_finish_optimization();
    }
    else if (is_waiting())
    {
      check_waiting_timeout();
    }
  }
  if (optimizer_state_publisher_->get_subscription_count() > 0)
  {
    cslam_common_interfaces::msg::OptimizerState state_msg;
    state_msg.state = optimizer_state_;
    optimizer_state_publisher_->publish(state_msg);
  }
}