cmake_minimum_required(VERSION 3.5)
project(cslam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(perception_pcl REQUIRED)
#find_package(eigen_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(message_filters REQUIRED)
find_package(class_loader REQUIRED)
#find_package(rosgraph_msgs REQUIRED)
find_package(image_geometry REQUIRED)
#find_package(pluginlib REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_conversions REQUIRED)

# Optional components
#find_package(costmap_2d)
#find_package(octomap_msgs)
#find_package(apriltag_msgs)
#find_package(find_object_2d)
#find_package(fiducial_msgs)

find_package(RTABMap 0.20.18 REQUIRED)

# Find GTSAM
find_package(GTSAM CONFIG REQUIRED)

# kinetic issue, rtabmap now requires at least c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.7 
            REQUIRED QUIET COMPONENTS 
            common 
            io 
            kdtree 
            search 
            surface 
            filters 
            registration 
            sample_consensus 
            segmentation
            ) #This crashes idl generation if all components are found?! see https://github.com/ros2/rosidl/issues/402#issuecomment-565586908

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RTABMap_INCLUDE_DIRS}
  ${rtabmap_ros_INCLUDE_DIRS}
  ${rtabmap_msgs_INCLUDE_DIRS}
  ${rtabmap_conversions_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# libraries
SET(Libraries
   pcl_conversions
   cv_bridge
   rclcpp
   rclcpp_components
   sensor_msgs
   std_msgs
   nav_msgs
   nav2_msgs
   geometry_msgs
   image_transport
   tf2
   tf2_eigen
   tf2_ros
   laser_geometry
   message_filters
   class_loader
   visualization_msgs
   image_geometry
   stereo_msgs
   diagnostic_msgs
   tf2_geometry_msgs
   pcl_ros
   pcl_msgs
   perception_pcl
   RTABMap
   rtabmap_ros
   rtabmap_msgs
   rtabmap_conversions
)

find_package(cslam_common_interfaces REQUIRED)
find_package(cslam_common_interfaces REQUIRED)

add_executable(map_manager 
  src/front_end/map_manager.cpp 
  src/front_end/map_manager_node.cpp 
  src/front_end/stereo_handler.cpp 
  src/front_end/rgbd_handler.cpp 
  #src/front_end/sensor_msg_utils.cpp
  src/front_end/visualization_utils.cpp)
add_dependencies(map_manager  
  map_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
ament_target_dependencies(map_manager 
  ${Libraries} 
  cslam_common_interfaces cslam_common_interfaces)
set_target_properties(map_manager 
  PROPERTIES OUTPUT_NAME "map_manager")

add_executable(pose_graph_manager 
  src/back_end/decentralized_pgo.cpp
  src/back_end/pose_graph_manager_node.cpp
  src/back_end/utils/logger.cpp
  src/back_end/gtsam_utils.cpp
  src/back_end/utils/simulated_rendezvous.cpp)
add_dependencies(pose_graph_manager  
pose_graph_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
ament_target_dependencies(pose_graph_manager 
  ${Libraries}
  cslam_common_interfaces cslam_common_interfaces)
target_link_libraries(pose_graph_manager gtsam)
set_target_properties(pose_graph_manager 
  PROPERTIES OUTPUT_NAME "pose_graph_manager")

# Only required when using messages built from the same package
# https://index.ros.org/doc/ros2/Tutorials/Rosidl-Tutorial/
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY 
   models
   DESTINATION share/${PROJECT_NAME}/
)

# Install nodes
install(
  TARGETS map_manager pose_graph_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS 
cslam/loop_closure_detection_node.py 
cslam/lidar_handler_node.py 
DESTINATION lib/${PROJECT_NAME})

# Add tests
find_package(ament_cmake_pytest REQUIRED)
set(_cslam_python_tests
      tests/test_algebraic_connectivity.py
      tests/test_sparse_matching.py
      tests/test_broker.py
  )
foreach(_test_path ${_cslam_python_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endforeach()

ament_package()