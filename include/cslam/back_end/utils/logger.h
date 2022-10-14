#ifndef _CSLAM_LOGGER_H_
#define _CSLAM_LOGGER_H_

#include <vector>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <ctime>
#include <chrono>
#include <iomanip>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cslam_common_interfaces/msg/pose_graph.hpp>

namespace cslam
{

    class Logger
    { // TODO: document
    public:
        Logger(std::shared_ptr<rclcpp::Node> &node, const unsigned int &robot_id, const unsigned int &nb_robots, const std::string &log_folder);

        void add_pose_graph_log_info(const cslam_common_interfaces::msg::PoseGraph &msg);

        void log_initial_global_pose_graph(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                           const gtsam::Values::shared_ptr &initial);

        void log_optimized_global_pose_graph(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                             const gtsam::Values &result,
                                             const unsigned int &origin_robot_id);

        void start_timer();

        void stop_timer();

        void write_logs();

    private:
    
        double compute_error(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                             const gtsam::Values::shared_ptr &result);

        std::vector<std::pair<std::pair<gtsam::LabeledSymbol, gtsam::LabeledSymbol>, double>> compute_inter_robot_loop_closure_errors(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                                 const gtsam::Values::shared_ptr &result);

        std::shared_ptr<rclcpp::Node> node_;

        std::string log_folder_;
        unsigned int robot_id_, origin_robot_id_, nb_robots_;
        std::chrono::steady_clock::time_point start_time_;
        uint64_t elapsed_time_, total_pgo_time_;

        std::map<unsigned int, std::map<unsigned int, sensor_msgs::msg::NavSatFix>> gps_values_;
        std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr> initial_global_pose_graph_;
        std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr> optimized_global_pose_graph_;
        std::vector<cslam_common_interfaces::msg::PoseGraph> pose_graphs_log_info_;
    };

} // namespace cslam

#endif // _CSLAM_LOGGER_H_