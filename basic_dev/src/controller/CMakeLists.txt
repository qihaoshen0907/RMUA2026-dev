cmake_minimum_required(VERSION 3.0.2)
project(controller_test)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS airsim_ros geometry_msgs message_runtime nav_msgs roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller_test
  src/controllerTest.cpp
  src/mpc_controller.cpp
)
target_link_libraries(controller_test ${catkin_LIBRARIES})
