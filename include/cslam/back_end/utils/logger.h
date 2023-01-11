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
#include <diagnostic_msgs/msg/key_value.hpp>

#include <cslam_common_interfaces/msg/pose_graph.hpp>
#include <cslam_common_interfaces/msg/inter_robot_matches.hpp>

namespace cslam
{

    class Logger
    { 
    /**
     * @brief Logger class to log various metrics
     * 
     */
    public:
        Logger(std::shared_ptr<rclcpp::Node> &node, const unsigned int &robot_id, const unsigned int &max_nb_robots, const std::string &log_folder);

        void add_pose_graph_log_info(const cslam_common_interfaces::msg::PoseGraph &msg);

        void log_initial_global_pose_graph(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                           const gtsam::Values::shared_ptr &initial);

        void log_optimized_global_pose_graph(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                             const gtsam::Values &result,
                                             const unsigned int &origin_robot_id);

        void start_timer();

        void stop_timer();

        void write_logs();

        /**
         * @brief Receive log messages
         * 
         * @param msg 
         */
        void log_callback(const diagnostic_msgs::msg::KeyValue::ConstSharedPtr msg);

        /**
         * @brief Receive log messages
         * 
         * @param msg 
         */
        void log_matches_callback(const cslam_common_interfaces::msg::InterRobotMatches::ConstSharedPtr msg);

        void fill_msg(cslam_common_interfaces::msg::PoseGraph & msg);

        void log_pose_timestamp(const gtsam::LabeledSymbol & symbol, const int& sec, const int& nanosec);

    private:
    
        double compute_error(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                             const gtsam::Values::shared_ptr &result);

        std::vector<std::pair<std::pair<gtsam::LabeledSymbol, gtsam::LabeledSymbol>, double>> compute_inter_robot_loop_closure_errors(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                                                 const gtsam::Values::shared_ptr &result);

        std::shared_ptr<rclcpp::Node> node_;

        std::string log_folder_;
        unsigned int robot_id_, origin_robot_id_, max_nb_robots_;
        std::chrono::steady_clock::time_point start_time_;
        uint64_t elapsed_time_, total_pgo_time_;

        std::map<unsigned int, std::map<unsigned int, sensor_msgs::msg::NavSatFix>> gps_values_;
        std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr> initial_global_pose_graph_;
        std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr> optimized_global_pose_graph_;
        std::vector<cslam_common_interfaces::msg::PoseGraph> pose_graphs_log_info_;

        rclcpp::Subscription<diagnostic_msgs::msg::KeyValue>::SharedPtr
            logger_subscriber_;

        rclcpp::Subscription<cslam_common_interfaces::msg::InterRobotMatches>::SharedPtr
            inter_robot_matches_subscriber_;

        cslam_common_interfaces::msg::InterRobotMatches spectral_matches_;

        unsigned int log_nb_matches_, log_nb_failed_matches_, log_nb_vertices_transmitted_, log_nb_matches_selected_, log_detection_cumulative_communication_, log_local_descriptors_cumulative_communication_;
        float log_sparsification_cumulative_computation_time_;

        std::map<gtsam::LabeledSymbol, std::pair<int, int>> pose_time_map_;
    };

} // namespace cslam

#endif // _CSLAM_LOGGER_H_