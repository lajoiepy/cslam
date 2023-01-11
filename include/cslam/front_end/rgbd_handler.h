#ifndef _RGBDHANDLER_H_
#define _RGBDHANDLER_H_

#include <rclcpp/rclcpp.hpp>

#include <rtabmap_ros/msg/rgbd_image.hpp>

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UStl.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cslam_common_interfaces/msg/keyframe_odom.hpp>
#include <cslam_common_interfaces/msg/keyframe_rgb.hpp>
#include <cslam_common_interfaces/msg/viz_point_cloud.hpp>
#include <cslam_common_interfaces/msg/inter_robot_loop_closure.hpp>
#include <cslam_common_interfaces/msg/local_descriptors_request.hpp>
#include <cslam_common_interfaces/msg/local_image_descriptors.hpp>
#include <cslam_common_interfaces/msg/local_keyframe_match.hpp>
#include <cslam_common_interfaces/msg/intra_robot_loop_closure.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <deque>
#include <functional>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>

#include <memory>

#include "cslam/front_end/sensor_msg_utils.h"
#include "cslam/front_end/sensor_handler_interface.h"
#include "cslam/front_end/visualization_utils.h"

namespace cslam
{

    class RGBDHandler : public ISensorHandler
    {
    public:
        /**
         * @brief Initialization of parameters and ROS 2 objects
         *
         * @param node ROS 2 node handle
         */
        RGBDHandler(std::shared_ptr<rclcpp::Node> &node);
        ~RGBDHandler(){};

        /**
         * @brief Processes Latest received image
         *
         */
        virtual void process_new_sensor_data();

        /**
         * @brief Service callback to publish local descriptors
         *
         * @param request Image ID to send and matching info
         */
        void local_descriptors_request(
            cslam_common_interfaces::msg::LocalDescriptorsRequest::
                ConstSharedPtr request);

        /**
         * @brief Receives a local match and tries to compute a local loop closure
         *
         * @param msg
         */
        void receive_local_keyframe_match(
            cslam_common_interfaces::msg::LocalKeyframeMatch::ConstSharedPtr
                msg);

        /**
         * @brief Message callback to receive descriptors and compute
         *
         * @param msg local descriptors
         */
        void receive_local_image_descriptors(
            const std::shared_ptr<
                cslam_common_interfaces::msg::LocalImageDescriptors>
                msg);

        /**
         * @brief Computes local 3D descriptors from frame data and store them
         *
         * @param frame_data Full frame data
         */
        void
        compute_local_descriptors(std::shared_ptr<rtabmap::SensorData> &frame_data);

        /**
         * @brief converts descriptors to sensore data
         *
         * @param msg local descriptors
         * @return rtabmap::SensorData&
         */
        virtual void local_descriptors_msg_to_sensor_data(
            const std::shared_ptr<
                cslam_common_interfaces::msg::LocalImageDescriptors>
                msg,
            rtabmap::SensorData &sensor_data);

        /**
         * @brief converts sensor data to descriptor msg
         *
         * @param sensor_data local descriptors
         * @param msg_data rtabmap_ros::msg::RGBDImage&
         */
        void sensor_data_to_rgbd_msg(
            const std::shared_ptr<rtabmap::SensorData> sensor_data,
            rtabmap_ros::msg::RGBDImage &msg_data);

        /**
         * @brief Generate a new keyframe according to the policy
         *
         * @param keyframe Sensor data
         * @return true A new keyframe is added to the map
         * @return false The frame is rejected
         */
        bool generate_new_keyframe(std::shared_ptr<rtabmap::SensorData> &keyframe);

        /**
         * @brief Function to send the image to the python node
         *
         * @param keypoints_data keyframe keypoints data
         */
        void send_keyframe(const std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<const nav_msgs::msg::Odometry>> &keypoints_data);

        /**
         * @brief Function to send the image to the python node
         *
         * @param keypoints_data keyframe keypoints data
         * @param gps_data GPS data
         */
        void send_keyframe(const std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<const nav_msgs::msg::Odometry>> &keypoints_data, const sensor_msgs::msg::NavSatFix& gps_data);

        /**
         * @brief Send keypoints for visualizations
         *
         * @param keypoints_data keyframe keypoints data
         */
        virtual void send_visualization(const std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<const nav_msgs::msg::Odometry>> &keypoints_data);

        /**
         * @brief Send keypoints for visualizations
         *
         * @param keypoints_data keyframe keypoints data
         */
        void send_visualization_keypoints(const std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<const nav_msgs::msg::Odometry>> &keypoints_data);

        /**
         * @brief Send colored pointcloud for visualizations
         *
         * @param sensor_data RGBD image
         */
        void send_visualization_pointcloud(const std::shared_ptr<rtabmap::SensorData> &sensor_data);

        /**
         * @brief Callback receiving sync data from camera
         *
         * @param image_rgb
         * @param image_depth
         * @param camera_info_rgb
         * @param camera_info_depth
         * @param odom
         */
        void rgbd_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr image_rect_rgb,
            const sensor_msgs::msg::Image::ConstSharedPtr image_rect_depth,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_rgb,
            const nav_msgs::msg::Odometry::ConstSharedPtr odom);

        /**
         * @brief Clear images and large data fields in sensor data
         *
         * @param sensor_data frame data
         */
        void clear_sensor_data(std::shared_ptr<rtabmap::SensorData> &sensor_data);

        /**
         * @brief GPS data callback
         *
         * @param msg
         */
        void gps_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

        /**
         * @brief Subsample pointcloud to reduce size for visualization
         * 
         * @param input_cloud 
         * @return pcl::PointCloud<pcl::PointXYZRGB>& 
         */
        sensor_msgs::msg::PointCloud2 visualization_pointcloud_voxel_subsampling(
                        const sensor_msgs::msg::PointCloud2 &input_cloud);

    protected:
        std::deque<std::pair<std::shared_ptr<rtabmap::SensorData>,
                             nav_msgs::msg::Odometry::ConstSharedPtr>>
            received_data_queue_;

        std::shared_ptr<rtabmap::SensorData> previous_keyframe_;

        std::map<int, std::shared_ptr<rtabmap::SensorData>> local_descriptors_map_;

        std::shared_ptr<rclcpp::Node> node_;

        unsigned int min_inliers_, max_nb_robots_, robot_id_, max_queue_size_,
            nb_local_keyframes_;

        message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odometry_;

        rclcpp::Subscription<
            cslam_common_interfaces::msg::LocalDescriptorsRequest>::SharedPtr
            send_local_descriptors_subscriber_;

        rclcpp::Publisher<
            cslam_common_interfaces::msg::LocalImageDescriptors>::SharedPtr
            local_descriptors_publisher_,
            visualization_local_descriptors_publisher_;

        rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeRGB>::SharedPtr
            keyframe_data_publisher_;

        rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr
            keyframe_odom_publisher_;

        rclcpp::Publisher<cslam_common_interfaces::msg::VizPointCloud>::SharedPtr
            keyframe_pointcloud_publisher_;

        rclcpp::Subscription<
            cslam_common_interfaces::msg::LocalKeyframeMatch>::SharedPtr
            local_keyframe_match_subscriber_;

        rclcpp::Subscription<
            cslam_common_interfaces::msg::LocalImageDescriptors>::SharedPtr
            local_descriptors_subscriber_;

        rtabmap::RegistrationVis registration_;

        rclcpp::Publisher<
            cslam_common_interfaces::msg::InterRobotLoopClosure>::SharedPtr
            inter_robot_loop_closure_publisher_;

        rclcpp::Publisher<
            cslam_common_interfaces::msg::IntraRobotLoopClosure>::SharedPtr
            intra_robot_loop_closure_publisher_;

        rclcpp::Publisher<
            diagnostic_msgs::msg::KeyValue>::SharedPtr
            log_publisher_;
        unsigned int log_total_local_descriptors_cumulative_communication_;
        bool enable_logs_;

        std::shared_ptr<tf2_ros::Buffer>
            tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string base_frame_id_;
        float keyframe_generation_ratio_threshold_;
        bool generate_new_keyframes_based_on_inliers_ratio_;

        unsigned int visualization_period_ms_;
        bool enable_visualization_;
        float visualization_voxel_size_, visualization_max_range_;

        bool enable_gps_recording_;
        std::string gps_topic_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
        sensor_msgs::msg::NavSatFix latest_gps_fix_;
        std::deque<sensor_msgs::msg::NavSatFix>
            received_gps_queue_;

    private:
        image_transport::SubscriberFilter sub_image_color_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_color_;
        image_transport::SubscriberFilter sub_image_depth_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_depth_;
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo,
            nav_msgs::msg::Odometry>
            RGBDSyncPolicy;
        message_filters::Synchronizer<RGBDSyncPolicy> *rgbd_sync_policy_;

        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_;
    };
} // namespace cslam
#endif
