cmake_minimum_required(VERSION 2.8.3)
project(calculate_imu_angle_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calculate_imu_angle_ros src/calculate_imu_angle_ros.cpp)
target_link_libraries(calculate_imu_angle_ros ${catkin_LIBRARIES})

install(TARGETS calculate_imu_angle_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

