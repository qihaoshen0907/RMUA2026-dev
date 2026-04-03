#!/bin/bash
cd /basic_dev
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 启动 basic_dev
rosrun basic_dev basic_dev &

# 启动 imu_gps_odometry
rosrun imu_gps_odometry imu_gps_odometry &

# 启动 controller_test（放前台，保持容器存活）
rosrun controller_test controller_test


