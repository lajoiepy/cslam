#include "cslam/back_end/pose_graph_manager.h"

using namespace cslam;

PoseGraphManager::PoseGraphManager(std::shared_ptr<rclcpp::Node> &node): node_(node), max_waiting_time_sec_(60,0) {

  node_->get_parameter("nb_robots", nb_robots_);
  node_->get_parameter("robot_id", robot_id_);
  node_->get_parameter("pose_graph_manager_process_period_ms", pose_graph_manager_process_period_ms_);
  node_->get_parameter("pose_graph_optimization_loop_period_ms", pose_graph_optimization_loop_period_ms_);

  int max_waiting_param;
  node_->get_parameter("max_waiting_time_sec", max_waiting_param);
  max_waiting_time_sec_ = rclcpp::Duration(max_waiting_param, 0);

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

  // Optimization timers
  optimization_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(pose_graph_manager_process_period_ms_), std::bind(&PoseGraphManager::optimization_callback, this));

  optimization_loop_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(pose_graph_optimization_loop_period_ms_), std::bind(&PoseGraphManager::optimization_loop_callback, this));

  // Publisher for optimization result
  optimization_result_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizationResult>(
          "optimization_result", 100);
          
  optimizer_state_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizerState>(
          "optimizer_state", 100);

  // Initialize inter-robot loop closures measurements
  for (unsigned int i = 0; i < nb_robots_; i++)
  {
    for (unsigned int j = i + 1; j < nb_robots_; j++)
    {
      inter_robot_loop_closures_.insert({{i, j}, std::vector<gtsam::BetweenFactor<gtsam::Pose3>>()});
    }
  }

  // Get neighbors ROS 2 objects
  get_current_neighbors_publisher_ =
    node->create_publisher<std_msgs::msg::String>("get_current_neighbors", 100);

  current_neighbors_subscriber_ = node->create_subscription<
        cslam_common_interfaces::msg::
                                      RobotIds>(
        "current_neighbors", 100,
        std::bind(&PoseGraphManager::current_neighbors_callback, this,
                    std::placeholders::_1));
  
  // PoseGraph ROS 2 objects
  for (unsigned int i = 0; i < nb_robots_; i++)
  {
    get_pose_graph_publishers_.insert({i,
      node->create_publisher<std_msgs::msg::String>("/r" + std::to_string(i) + "/get_pose_graph", 100)});
    received_pose_graphs_.insert({i, false});
  }

  get_pose_graph_subscriber_ = node->create_subscription<
        std_msgs::msg::String>(
        "get_pose_graph", 100,
        std::bind(&PoseGraphManager::get_pose_graph_callback, this,
                    std::placeholders::_1));

  pose_graph_publisher_ =
    node->create_publisher<cslam_common_interfaces::msg::PoseGraph>("/pose_graph", 100);

  pose_graph_subscriber_ = node->create_subscription<
        cslam_common_interfaces::msg::PoseGraph>(
        "/pose_graph", 100,
        std::bind(&PoseGraphManager::pose_graph_callback, this,
                    std::placeholders::_1));

  // Optimizer
  optimizer_state_ = OptimizerState::IDLE;
  is_waiting_ = false;

  // Add prior 
  // TODO: not for decentralized
  //gtsam::LabeledSymbol first_symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), 0);
  //pose_graph_->addPrior(first_symbol, gtsam::Pose3(), default_noise_model_);
}

void PoseGraphManager::reinitialize_received_pose_graphs(){
  for (unsigned int i = 0; i < nb_robots_; i ++)
  {
    received_pose_graphs_[i] = false;
  }
  other_robots_graph_and_estimates_.clear();
}

bool PoseGraphManager::check_received_pose_graphs(){
  bool received_all = true;
  for (auto id : current_robot_ids_.ids)
  {
    received_all &= received_pose_graphs_[id];
  }
  return received_all;
}

void PoseGraphManager::odometry_callback(const cslam_common_interfaces::msg::KeyframeOdom::ConstSharedPtr msg) {

  gtsam::Pose3 current_estimate = odometry_msg_to_pose3(msg->odom);
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
      gtsam::Pose3 measurement = transform_msg_to_pose3(msg->transform);

      unsigned char robot0_c = ROBOT_LABEL(msg->robot0_id);
      gtsam::LabeledSymbol symbol_from(GRAPH_LABEL, robot0_c, msg->robot0_image_id);
      unsigned char robot1_c = ROBOT_LABEL(msg->robot1_id);
      gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, robot1_c, msg->robot1_image_id);

      gtsam::BetweenFactor<gtsam::Pose3> factor(symbol_from, symbol_to, measurement, default_noise_model_);
      
      inter_robot_loop_closures_[{std::min(robot0_c, robot1_c), std::max(robot0_c, robot1_c)}].emplace_back(factor);
    }
  }

void PoseGraphManager::current_neighbors_callback(const cslam_common_interfaces::msg::
                                      RobotIds::ConstSharedPtr msg)
{
  current_robot_ids_ = *msg;
  optimizer_state_ = OptimizerState::POSEGRAPH_COLLECTION;
  end_waiting();
}

void PoseGraphManager::get_pose_graph_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  cslam_common_interfaces::msg::PoseGraph out_msg;
  out_msg.robot_id = robot_id_;
  out_msg.values = gtsam_values_to_msg(current_pose_estimates_);
  out_msg.edges = gtsam_factors_to_msg(pose_graph_);
  pose_graph_publisher_->publish(out_msg);
}

void PoseGraphManager::pose_graph_callback(const cslam_common_interfaces::msg::
                                      PoseGraph::ConstSharedPtr msg)
{
  other_robots_graph_and_estimates_.insert({msg->robot_id, {edges_msg_to_gtsam(msg->edges), values_msg_to_gtsam(msg->values)}});
  received_pose_graphs_[msg->robot_id] = true;
  if (check_received_pose_graphs())
  {
    end_waiting();
    optimizer_state_ = OptimizerState::OPTIMIZATION;
  }
}

void PoseGraphManager::resquest_current_neighbors(){
  get_current_neighbors_publisher_->publish(std_msgs::msg::String());  
}

void PoseGraphManager::start_waiting(){
  optimizer_state_ = OptimizerState::WAITING;
  is_waiting_ = true;
  start_waiting_time_ = node_->now();
}

void PoseGraphManager::end_waiting(){
  is_waiting_ = false;
}

bool PoseGraphManager::check_waiting_timeout(){
  if ((node_->now() - start_waiting_time_) > max_waiting_time_sec_)
  {
    end_waiting();
    optimizer_state_ = OptimizerState::IDLE;
  }
  return is_waiting_;
}

void PoseGraphManager::optimization_callback(){
  if (optimizer_state_ == OptimizerState::IDLE){
      reinitialize_received_pose_graphs();
      resquest_current_neighbors();
      start_waiting();
  }
}

void PoseGraphManager::perform_optimization(){
    // TODO: Aggregate graphs

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

void PoseGraphManager::optimization_loop_callback(){
  if (!current_pose_estimates_->empty())
  {
    if (optimizer_state_ == OptimizerState::POSEGRAPH_COLLECTION) // TODO: Document
    {
      if (current_robot_ids_.ids.size() > 0)
      {
        for (auto id : current_robot_ids_.ids)
        {
          get_pose_graph_publishers_[id]->publish(std_msgs::msg::String());
        }
        start_waiting();
      }
      else
      {
        optimizer_state_ = OptimizerState::IDLE;
      }
    }
    else if (optimizer_state_ == OptimizerState::OPTIMIZATION)
    {
      // Call optimization
      perform_optimization();
      // TODO: Send updates
      optimizer_state_ = OptimizerState::IDLE;
    }
    else if (optimizer_state_ == OptimizerState::WAITING)
    {
      check_waiting_timeout();
    }
  }
  cslam_common_interfaces::msg::OptimizerState state_msg;
  state_msg.state = optimizer_state_;
  optimizer_state_publisher_->publish(state_msg);
}