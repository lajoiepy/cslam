#ifndef _STEREOHANDLER_H_
#define _STEREOHANDLER_H_

#include "cslam/front_end/rgbd_handler.h"

namespace cslam
{

    class StereoHandler : public RGBDHandler
    {
    public:
        /**
         * @brief Initialization of parameters and ROS 2 objects
         *
         * @param node ROS 2 node handle
         */
        StereoHandler(std::shared_ptr<rclcpp::Node> &node);
        ~StereoHandler(){};

        /**
         * @brief Callback receiving sync data from camera
         *
         * @param image_rect_left
         * @param image_rect_right
         * @param camera_info_left
         * @param camera_info_right
         * @param odom
         */
        void stereo_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr image_rect_left,
            const sensor_msgs::msg::Image::ConstSharedPtr image_rect_right,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_left,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_right,
            const nav_msgs::msg::Odometry::ConstSharedPtr odom);

        /**
         * @brief converts descriptors to sensore data
         *
         * @param msg local descriptors
         * @return rtabmap::SensorData&
         */
        virtual void local_descriptors_msg_to_sensor_data(
            const std::shared_ptr<cslam_common_interfaces::msg::LocalImageDescriptors> msg,
            rtabmap::SensorData &sensor_data);

        /**
         * @brief Send keypoints for visualizations
         * 
         * @param keypoints_data keyframe keypoints data
         */
        virtual void send_visualization(const std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<const nav_msgs::msg::Odometry>> &keypoints_data);

    private:
        image_transport::SubscriberFilter sub_image_rect_left_;
        image_transport::SubscriberFilter sub_image_rect_right_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_left_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_right_;
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
            nav_msgs::msg::Odometry>
            StereoSyncPolicy;
        message_filters::Synchronizer<StereoSyncPolicy> *stereo_sync_policy_;
    };
} // namespace cslam
#endif