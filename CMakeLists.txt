cmake_minimum_required(VERSION 2.8.3)
project(mono_vo_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  image_transport
)

find_package(OpenCV 3.0 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mono_vo_ros
#  CATKIN_DEPENDS OpenCV cv_bridge roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  image_calibrator
  src/image_calibrator_main.cpp
  src/ImageCalibrator.cpp
)

target_link_libraries(
  image_calibrator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(
  mono_vo_ros
  src/mono_vo_ros_main.cpp
  src/VisualOdometer.cpp
  src/RotationConverter.cpp
)

target_link_libraries(
  mono_vo_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
