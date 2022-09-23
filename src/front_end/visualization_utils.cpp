#include "cslam/front_end/visualization_utils.h"

namespace cslam
{

// Conversions implementations are from https://github.com/ros-perception/image_pipeline/tree/foxy/depth_image_proc

sensor_msgs::msg::PointCloud2 create_colored_pointcloud(const std::shared_ptr<rtabmap::SensorData> & sensor_data, const std_msgs::msg::Header & header)
{
  // Update camera model
  image_geometry::PinholeCameraModel model;
  sensor_msgs::msg::CameraInfo info_msg;
  rtabmap_ros::cameraModelToROS(sensor_data->cameraModels()[0], info_msg);
  model.fromCameraInfo(info_msg);

  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header = header;
  cloud_msg->height = sensor_data->depthRaw().rows;
  cloud_msg->width = sensor_data->depthRaw().cols;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  if (sensor_data->depthRaw().cols != sensor_data->imageRaw().cols || sensor_data->depthRaw().rows != sensor_data->imageRaw().rows) {
    RCLCPP_WARN(
      rclcpp::get_logger("cslam"),
      "Depth image size (%dx%d) doesn't match RGB image size (%dx%d)!",
      sensor_data->depthRaw().cols, sensor_data->depthRaw().rows, sensor_data->imageRaw().cols, sensor_data->imageRaw().rows);
    cloud_msg->height = 0;
    cloud_msg->width = 0;
    return *cloud_msg;
  }

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  // Convert Depth Image to Pointcloud
  if (sensor_data->depthRaw().type() == CV_16UC1) {
    depth_image_to_pointcloud<uint16_t>(sensor_data, cloud_msg, model);
  } else if (sensor_data->depthRaw().type() == CV_32FC1) {
    depth_image_to_pointcloud<float>(sensor_data, cloud_msg, model);
  } else {
    // Depth image has unsupported encoding
    return *cloud_msg;
  }

  // Add colors to pointcloud
  if (sensor_data->imageRaw().type() == CV_8UC3) {
    add_rgb_to_pointcloud(sensor_data, cloud_msg);
  } else if (sensor_data->imageRaw().type() == CV_8UC1) {
    add_rgb_to_pointcloud(sensor_data, cloud_msg);
  } else {
    // RGB image has unsupported encoding
    return *cloud_msg;
  }
  return *cloud_msg;
}

// Handles float or uint16 depths
template<typename T>
void depth_image_to_pointcloud(
  const std::shared_ptr<rtabmap::SensorData> & sensor_data,
  sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
  const image_geometry::PinholeCameraModel & model,
  double range_max)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters(T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T * depth_row = reinterpret_cast<const T *>(&sensor_data->depthRaw().data[0]);
  int row_step = sensor_data->depthRaw().step / sizeof(T);
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, depth_row += row_step) {
    for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!depth_image_proc::DepthTraits<T>::valid(depth)) {
        if (range_max != 0.0) {
          depth = depth_image_proc::DepthTraits<T>::fromMeters(range_max);
        } else {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);
    }
  }
}

void add_rgb_to_pointcloud(
  const std::shared_ptr<rtabmap::SensorData> & sensor_data,
  sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg)
{
  // Supported color encodings: BGR8, MONO8
  int red_offset, green_offset, blue_offset, color_step;
  if (sensor_data->imageRaw().type() == CV_8UC3) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 3;
  } else if (sensor_data->imageRaw().type() == CV_8UC1) {
    red_offset = 0;
    green_offset = 0;
    blue_offset = 0;
    color_step = 1;
  } else {
    return;
  }

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  const uint8_t * rgb = &sensor_data->imageRaw().data[0];
  int rgb_skip = sensor_data->imageRaw().step - sensor_data->imageRaw().cols * color_step;
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, rgb += rgb_skip) {
    for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u,
      rgb += color_step, ++iter_r, ++iter_g, ++iter_b)
    {
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}

} // namespace cslam