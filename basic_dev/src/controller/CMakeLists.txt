cmake_minimum_required(VERSION 3.0.2)
project(controller_test)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  geometry_msgs
  roscpp
  sensor_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS airsim_ros geometry_msgs roscpp sensor_msgs tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller_test src/controllerTest.cpp)
target_link_libraries(controller_test
  ${catkin_LIBRARIES}
)
