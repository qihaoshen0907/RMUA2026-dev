cmake_minimum_required(VERSION 3.0.2)

project(controller_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  roslib
  airsim_ros
  rpg_mpc
  quadrotor_common
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    roslib
    airsim_ros
    rpg_mpc
    quadrotor_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../rpg_mpc/model/quadrotor_mpc_codegen
  ${CMAKE_CURRENT_SOURCE_DIR}/../rpg_mpc/externals/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/../rpg_mpc/externals/qpoases/INCLUDE
)

add_executable(controller_test
  src/controllerTest.cpp
)

add_dependencies(controller_test
  ${catkin_EXPORTED_TARGETS}
  ${airsim_ros_EXPORTED_TARGETS}
)

target_link_libraries(controller_test
  ${catkin_LIBRARIES}
  mpc_wrapper
  mpc_solver
)
