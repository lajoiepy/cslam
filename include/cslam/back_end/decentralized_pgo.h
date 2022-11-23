#ifndef _DECENTRALIZED_PGO_H_
#define _DECENTRALIZED_PGO_H_

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cslam_common_interfaces/msg/keyframe_odom.hpp>
#include <cslam_common_interfaces/msg/optimization_result.hpp>
#include <cslam_common_interfaces/msg/optimizer_state.hpp>
#include <cslam_common_interfaces/msg/reference_frames.hpp>
#include <cslam_common_interfaces/msg/robot_ids_and_origin.hpp>
#include <cslam_common_interfaces/msg/inter_robot_loop_closure.hpp>
#include <cslam_common_interfaces/msg/intra_robot_loop_closure.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <future>
#include <gtsam/slam/dataset.h>
#include <list>
#include <thread>

#include "cslam/back_end/gtsam_utils.h"
#include "cslam/back_end/utils/logger.h"
#include "cslam/back_end/utils/simulated_rendezvous.h"

namespace cslam
{

    /**
     * @brief State of the Pose Graph Optimization node
     *
     * IDLE: Do nothing.
     * WAITING_FOR_NEIGHBORS_INFO: Wait for info on neighbors in range.
     * POSEGRAPH_COLLECTION: Send request for neighboring robots pose graph
     * WAITING_FOR_NEIGHBORS_POSEGRAPHS: Waits for neighboring robot pose graphs
     * START_OPTIMIZATION: Triggers optimization process.
     * OPTIMIZATION: While the pose graph optimization thresad is running.
     *
     */
    enum OptimizerState
    {
        IDLE,
        WAITING_FOR_NEIGHBORS_INFO,
        POSEGRAPH_COLLECTION,
        WAITING_FOR_NEIGHBORS_POSEGRAPHS,
        START_OPTIMIZATION,
        OPTIMIZATION
    };

    class DecentralizedPGO
    {
    public:
        /**
         * @brief Initialization of parameters and ROS 2 objects
         *
         * @param node ROS 2 node handle
         */
        DecentralizedPGO(std::shared_ptr<rclcpp::Node> &node);
        ~DecentralizedPGO(){};

        /**
         * @brief Receives odometry msg + keyframe id
         *
         * @param msg
         */
        void odometry_callback(
            const cslam_common_interfaces::msg::KeyframeOdom::ConstSharedPtr msg);

        /**
         * @brief Receives inter-robot loop closures
         *
         * @param msg
         */
        void inter_robot_loop_closure_callback(
            const cslam_common_interfaces::msg::InterRobotLoopClosure::
                ConstSharedPtr msg);

        /**
         * @brief Receives intra-robot loop closures
         *
         * @param msg
         */
        void intra_robot_loop_closure_callback(
            const cslam_common_interfaces::msg::IntraRobotLoopClosure::
                ConstSharedPtr msg);

        /**
         * @brief Receives current neighbors
         *
         * @param msg
         */
        void current_neighbors_callback(
            const cslam_common_interfaces::msg::RobotIdsAndOrigin::ConstSharedPtr
                msg);

        /**
         * @brief Fill a PoseGraph message with the local data
         *
         * @return cslam_common_interfaces::msg::PoseGraph
         */
        cslam_common_interfaces::msg::PoseGraph fill_pose_graph_msg();

        /**
         * @brief Fill a PoseGraph message with the local data
         *
         * @return cslam_common_interfaces::msg::PoseGraph
         */
        cslam_common_interfaces::msg::PoseGraph fill_pose_graph_msg(const cslam_common_interfaces::msg::RobotIds &msg);

        /**
         * @brief Receives request for pose graph
         *
         * @param msg
         */
        void get_pose_graph_callback(
            const cslam_common_interfaces::msg::RobotIds::ConstSharedPtr msg);

        /**
         * @brief Receives pose graph
         *
         * @param msg
         */
        void pose_graph_callback(
            const cslam_common_interfaces::msg::PoseGraph::ConstSharedPtr msg);

        /**
         * @brief Receives estimates from the optimizer
         *
         * @param msg
         */
        void optimized_estimates_callback(
            const cslam_common_interfaces::msg::OptimizationResult::ConstSharedPtr
                msg);

        /**
         * @brief Prints current estimates
         *
         * @param msg
         */
        void write_current_estimates_callback(
            const std_msgs::msg::String::ConstSharedPtr msg);

        /**
         * @brief Starts pose graph optimization process every X ms (defined in
         * config)
         *
         */
        void optimization_callback();

        /**
         * @brief Step execution of the optimization State Machine
         *
         */
        void optimization_loop_callback();

        /**
         * @brief Update transform to origin
         *
         */
        void update_transform_to_origin(const gtsam::Pose3 &pose);

        /**
         * @brief Broadcast tf to current origin point
         *
         */
        void broadcast_tf_callback();

        /**
         * @brief Performs pose graph optimization
         *
         */
        void start_optimization();

        /**
         * @brief Request to check the current neighbors
         *
         */
        void resquest_current_neighbors();

        /**
         * @brief Sets received pose graphs to false
         *
         */
        void reinitialize_received_pose_graphs();

        /**
         * @brief Check if received all pose graphs
         *
         */
        bool check_received_pose_graphs();

        /**
         * @brief Start waiting
         *
         */
        void start_waiting();

        /**
         * @brief Check waiting timeout
         *
         */
        bool check_waiting_timeout();

        /**
         * @brief End waiting
         *
         */
        void end_waiting();

        /**
         * @brief Check if waiting
         *
         */
        bool is_waiting();

        /**
         * @brief Breadth First Seach to check connectivity
         *
         */
        std::map<unsigned int, bool> connected_robot_pose_graph();

        /**
         * @brief Put all connected pose graphs into one
         *
         */
        std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>
        aggregate_pose_graphs();

        /**
         * @brief Shares optimizes estimates with the robots concerned
         *
         */
        void share_optimized_estimates(const gtsam::Values &estimates);

        /**
         * @brief Publish heartbeat message periodically
         *
         */
        void heartbeat_timer_callback();

        /**
         * @brief Publish heartbeat message periodically
         *
         */
        void visualization_callback();

        /**
         * @brief Check if the local robot is the optimizer according to the specified
         * priority rule Default priority rule: lowest ID
         *
         * @return true
         * @return false
         */
        bool is_optimizer();

        /**
         * @brief Pose graph optimization function
         *
         * @param graph pose graph
         * @param initial initial values
         * @return gtsam::Values optimized values
         */
        gtsam::Values optimize(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                               const gtsam::Values::shared_ptr &initial);

        /**
         * @brief Check if the optimization is finished
         *
         */
        void check_result_and_finish_optimization();

    private:
        std::shared_ptr<rclcpp::Node> node_;

        unsigned int max_nb_robots_, robot_id_, optimization_count_;
        bool enable_logs_;

        unsigned int pose_graph_optimization_start_period_ms_,
            pose_graph_optimization_loop_period_ms_,
            visualization_period_ms_;

        bool enable_visualization_;

        gtsam::SharedNoiseModel default_noise_model_;
        float rotation_default_noise_std_, translation_default_noise_std_;

        gtsam::NonlinearFactorGraph::shared_ptr pose_graph_;
        gtsam::Values::shared_ptr current_pose_estimates_;
        gtsam::Values::shared_ptr odometry_pose_estimates_;

        std::map<unsigned int, std::pair<gtsam::NonlinearFactorGraph::shared_ptr,
                                         gtsam::Values::shared_ptr>>
            other_robots_graph_and_estimates_;

        gtsam::Pose3 latest_local_pose_, local_pose_at_latest_optimization_,
            tentative_local_pose_at_latest_optimization_, latest_optimized_pose_;
        gtsam::LabeledSymbol latest_local_symbol_;

        bool enable_pose_timestamps_recording_;

        std::map<std::pair<unsigned int, unsigned int>,
                 std::vector<gtsam::BetweenFactor<gtsam::Pose3>>>
            inter_robot_loop_closures_;

        rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr
            odometry_subscriber_;

        rclcpp::Subscription<
            cslam_common_interfaces::msg::InterRobotLoopClosure>::SharedPtr
            inter_robot_loop_closure_subscriber_;

        rclcpp::Subscription<
            cslam_common_interfaces::msg::IntraRobotLoopClosure>::SharedPtr
            intra_robot_loop_closure_subscriber_;

        rclcpp::Publisher<cslam_common_interfaces::msg::OptimizationResult>::SharedPtr
            debug_optimization_result_publisher_;

        rclcpp::Subscription<cslam_common_interfaces::msg::OptimizationResult>::
            SharedPtr optimized_estimates_subscriber_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
            optimized_pose_estimate_publisher_;

        std::map<unsigned int,
                 rclcpp::Publisher<
                     cslam_common_interfaces::msg::OptimizationResult>::SharedPtr>
            optimized_estimates_publishers_;

        rclcpp::TimerBase::SharedPtr optimization_timer_, optimization_loop_timer_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
            get_current_neighbors_publisher_;

        rclcpp::Subscription<cslam_common_interfaces::msg::RobotIdsAndOrigin>::
            SharedPtr current_neighbors_subscriber_;

        std::map<unsigned int,
                 rclcpp::Publisher<cslam_common_interfaces::msg::RobotIds>::SharedPtr>
            get_pose_graph_publishers_;

        std::map<unsigned int, bool> received_pose_graphs_;

        std::set<unsigned int> connected_robots_;

        std::map<unsigned int, std::vector<unsigned int>>
            received_pose_graphs_connectivity_;

        rclcpp::Subscription<cslam_common_interfaces::msg::RobotIds>::SharedPtr
            get_pose_graph_subscriber_;

        rclcpp::Subscription<cslam_common_interfaces::msg::PoseGraph>::SharedPtr
            pose_graph_subscriber_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
            write_current_estimates_subscriber_;

        rclcpp::Publisher<cslam_common_interfaces::msg::PoseGraph>::SharedPtr
            pose_graph_publisher_;

        rclcpp::Publisher<cslam_common_interfaces::msg::PoseGraph>::SharedPtr
            visualization_pose_graph_publisher_;

        cslam_common_interfaces::msg::RobotIdsAndOrigin current_neighbors_ids_;

        OptimizerState optimizer_state_;

        std::future<gtsam::Values> optimization_result_;

        gtsam::GraphAndValues aggregate_pose_graph_;

        rclcpp::Publisher<cslam_common_interfaces::msg::OptimizerState>::SharedPtr
            optimizer_state_publisher_;

        bool is_waiting_;

        std::string log_folder_;

        rclcpp::Time start_waiting_time_;
        rclcpp::Duration max_waiting_time_sec_;

        unsigned int origin_robot_id_;
        double heartbeat_period_sec_;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr heartbeat_publisher_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::TimerBase::SharedPtr heartbeat_timer_, tf_broadcaster_timer_,
            visualization_timer_;

        geometry_msgs::msg::TransformStamped origin_to_first_pose_;

        bool enable_broadcast_tf_frames_;

        rclcpp::Publisher<cslam_common_interfaces::msg::ReferenceFrames>::SharedPtr
            reference_frame_per_robot_publisher_;

        std::shared_ptr<Logger> logger_;

        std::map<unsigned int, sensor_msgs::msg::NavSatFix> gps_data_;
        bool enable_gps_recording_;

        bool enable_simulated_rendezvous_;
        std::shared_ptr<SimulatedRendezVous> sim_rdv_;
    };
} // namespace cslam
#endif