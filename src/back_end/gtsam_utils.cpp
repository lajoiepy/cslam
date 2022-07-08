#include "cslam/back_end/gtsam_utils.h"

namespace cslam {

geometry_msgs::msg::Pose gtsam_pose_to_msg(const gtsam::Pose3 &pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  msg.position.z = pose.z();
  auto rotation = pose.rotation().quaternion();
  msg.orientation.w = rotation[0];
  msg.orientation.x = rotation[1];
  msg.orientation.y = rotation[2];
  msg.orientation.z = rotation[3];
  return msg;
}

geometry_msgs::msg::Transform
gtsam_pose_to_transform_msg(const gtsam::Pose3 &pose) {
  geometry_msgs::msg::Transform msg;
  msg.translation.x = pose.x();
  msg.translation.y = pose.y();
  msg.translation.z = pose.z();

  auto rotation = pose.rotation().quaternion();
  msg.rotation.w = rotation[0];
  msg.rotation.x = rotation[1];
  msg.rotation.y = rotation[2];
  msg.rotation.z = rotation[3];

  return msg;
}

std::vector<cslam_common_interfaces::msg::PoseGraphValue>
gtsam_values_to_msg(const gtsam::Values &values) {
  std::vector<cslam_common_interfaces::msg::PoseGraphValue> poses;
  for (const auto key_value : values) {
    auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(
        &key_value.value);
    if (!p)
      continue;
    cslam_common_interfaces::msg::PoseGraphValue pose_msg;
    pose_msg.pose = gtsam_pose_to_msg(p->value());
    auto key = gtsam::LabeledSymbol(key_value.key);
    pose_msg.key.robot_id = ROBOT_ID(key.label());
    pose_msg.key.keyframe_id = key.index();

    poses.emplace_back(pose_msg);
  }
  return poses;
}

std::vector<cslam_common_interfaces::msg::PoseGraphValue>
gtsam_values_to_msg(const gtsam::Values::shared_ptr values) {
  return gtsam_values_to_msg(*values);
}

std::vector<cslam_common_interfaces::msg::PoseGraphEdge>
gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph &factors) {
  std::vector<cslam_common_interfaces::msg::PoseGraphEdge> edges;
  for (const auto &factor_ : factors) {
    auto factor =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
            factor_);
    if (factor) {
      cslam_common_interfaces::msg::PoseGraphEdge edge_msg;

      auto key_from = gtsam::LabeledSymbol(factor->key1());
      auto key_to = gtsam::LabeledSymbol(factor->key2());
      edge_msg.key_from.robot_id = ROBOT_ID(key_from.label());
      edge_msg.key_from.keyframe_id = key_from.index();
      edge_msg.key_to.robot_id = ROBOT_ID(key_to.label());
      edge_msg.key_to.keyframe_id = key_to.index();

      edge_msg.measurement = gtsam_pose_to_msg(factor->measured());

      gtsam::SharedNoiseModel model = factor->noiseModel();
      auto noise =
          boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(model);
      auto sigmas = noise->sigmas();
      for (unsigned int i = 0; i < 6; i++) {
        edge_msg.noise_std[i] = sigmas[i];
      }

      edges.emplace_back(edge_msg);
    }
  }
  return edges;
}

std::vector<cslam_common_interfaces::msg::PoseGraphEdge>
gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph::shared_ptr factors) {
  return gtsam_factors_to_msg(*factors);
}

gtsam::Pose3 transform_msg_to_pose3(const geometry_msgs::msg::Transform &msg) {
  gtsam::Rot3 rotation(msg.rotation.w, msg.rotation.x, msg.rotation.y,
                       msg.rotation.z);
  return gtsam::Pose3(
      rotation, {msg.translation.x, msg.translation.y, msg.translation.z});
}

gtsam::Pose3 pose_msg_to_gtsam(const geometry_msgs::msg::Pose &pose) {
  gtsam::Rot3 rotation(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  return gtsam::Pose3(rotation,
                      {pose.position.x, pose.position.y, pose.position.z});
}

gtsam::Pose3 odometry_msg_to_pose3(const nav_msgs::msg::Odometry &odom_msg) {
  return pose_msg_to_gtsam(odom_msg.pose.pose);
}

gtsam::Values::shared_ptr values_msg_to_gtsam(
    const std::vector<cslam_common_interfaces::msg::PoseGraphValue> &msg) {
  auto values = boost::make_shared<gtsam::Values>();
  for (auto v : msg) {
    gtsam::Pose3 pose = pose_msg_to_gtsam(v.pose);
    gtsam::LabeledSymbol symbol(GRAPH_LABEL, ROBOT_LABEL(v.key.robot_id),
                                v.key.keyframe_id);
    values->insert(symbol, pose);
  }
  return values;
}

gtsam::NonlinearFactorGraph::shared_ptr edges_msg_to_gtsam(
    const std::vector<cslam_common_interfaces::msg::PoseGraphEdge> &msg) {
  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  for (auto e : msg) {
    gtsam::Pose3 pose = pose_msg_to_gtsam(e.measurement);
    gtsam::LabeledSymbol symbol_from(
        GRAPH_LABEL, ROBOT_LABEL(e.key_from.robot_id), e.key_from.keyframe_id);
    gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, ROBOT_LABEL(e.key_to.robot_id),
                                   e.key_to.keyframe_id);

    Eigen::VectorXd sigmas(6);
    sigmas << e.noise_std[0], e.noise_std[1], e.noise_std[2], e.noise_std[3],
        e.noise_std[4], e.noise_std[5];
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    gtsam::BetweenFactor<gtsam::Pose3> factor(symbol_from, symbol_to, pose,
                                              noise_model);
    graph->push_back(factor);
  }
  return graph;
}

} // namespace cslam