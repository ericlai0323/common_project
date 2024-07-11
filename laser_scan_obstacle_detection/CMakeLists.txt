cmake_minimum_required(VERSION 3.0.2)
project(laser_scan_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_scan_obstacle_detection src/laser_scan_obstacle_detection.cpp)
target_link_libraries(laser_scan_obstacle_detection ${catkin_LIBRARIES})
