#ifndef _VISUALIZATION_UTILS_H_
#define _VISUALIZATION_UTILS_H_

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rtabmap/core/SensorData.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cslam_common_interfaces/msg/keyframe_odom.hpp>
#include <deque>
#include <functional>
#include <thread>

#include <memory>

#include "cslam/front_end/sensor_msg_utils.h"
#include "cslam/front_end/utils/depth_traits.h"

namespace cslam
{
    // TODO: document
    sensor_msgs::msg::PointCloud2 create_colored_pointcloud(const std::shared_ptr<rtabmap::SensorData> &sensor_data, const std_msgs::msg::Header &header);

    template <typename T>
    void depth_image_to_pointcloud(
        const std::shared_ptr<rtabmap::SensorData> &sensor_data,
        sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg,
        const image_geometry::PinholeCameraModel &model,
        double range_max = 0.0);

    void add_rgb_to_pointcloud(
        const std::shared_ptr<rtabmap::SensorData> &sensor_data,
        sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg);

} // namespace cslam
#endif