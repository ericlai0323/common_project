cmake_minimum_required(VERSION 3.0.2)
project(odometry_to_tf)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
