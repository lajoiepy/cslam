cmake_minimum_required(VERSION 2.8.3)
project(external_loop_closure_detection)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs
             image_transport tf tf_conversions tf2_ros 
             nodelet message_filters class_loader rosgraph_msgs
             genmsg stereo_msgs image_geometry pluginlib
)

# Optional components
find_package(rtabmap_ros REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(RTABMap REQUIRED)

find_package(OpenCV REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   DetectLoopClosure.srv
 )

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES external_loop_closure_detection
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs
                 image_transport tf tf_conversions tf2_ros 
                 nodelet message_filters class_loader rosgraph_msgs
                 stereo_msgs image_geometry ${optional_dependencies}
  DEPENDS RTABMap OpenCV rtabmap_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  #${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RTABMap_INCLUDE_DIRS}
  ${rtabmap_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   ${RTABMap_LIBRARIES}
   ${rtabmap_ros_LIBRARIES}
)

add_executable(external_loop_closure_detection src/ExternalLoopClosureDetection.cpp)
target_link_libraries(external_loop_closure_detection ${Libraries})
set_target_properties(external_loop_closure_detection PROPERTIES OUTPUT_NAME "external_loop_detection")

catkin_install_python(PROGRAMS
  scripts/loop_closure_detection.py
  src/external_loop_closure_detection/nearest_neighbors.py
  src/external_loop_closure_detection/netvlad_loop_closure_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   launch/external_loop_closure_detection.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)