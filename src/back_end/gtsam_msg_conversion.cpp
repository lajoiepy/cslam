#include "cslam/back_end/gtsam_msg_conversion.h"

using namespace cslam;

geometry_msgs::msg::Pose gtsam_pose_to_msg(const gtsam::Pose3& pose)
{

}

std::vector<cslam_common_interfaces::msg::PoseGraphValue> gtsam_values_to_msg(const gtsam::Values::shared_ptr values)
{

}

std::vector<cslam_common_interfaces::msg::PoseGraphEdge> gtsam_factors_to_msg(const gtsam::NonlinearFactorGraph::shared_ptr factors)
{
    
}
