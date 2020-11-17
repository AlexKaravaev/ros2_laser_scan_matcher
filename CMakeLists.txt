cmake_minimum_required(VERSION 3.5)

project(ros2_laser_scan_matcher)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)
include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

add_executable(laser_scan_matcher src/laser_scan_matcher.cpp)
ament_target_dependencies(laser_scan_matcher rclcpp rcutils sensor_msgs)
target_include_directories(laser_scan_matcher
  PRIVATE
  "include"
)

ament_target_dependencies(laser_scan_matcher
  rclcpp
  sensor_msgs
  tf2_ros
  csm
)

install(TARGETS laser_scan_matcher DESTINATION lib/{PROJECT_NAME})


ament_package()