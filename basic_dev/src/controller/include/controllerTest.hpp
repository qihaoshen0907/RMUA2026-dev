#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/RotorPWM.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/PoseStamped.h>
#include <time.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <random>
#include "PDcontroller.hpp"
#include <fstream>


ros::ServiceClient g_takeoff_client;
std::vector<std::vector<Eigen::Vector3d>> globalPaths;
std::vector<Eigen::Vector3d> globalPath;
UAVLinearController g_PDcontroller;
std::random_device rd{};
std::mt19937 gen{rd()};
std::normal_distribution<double> gauss_dist{0.0, 1.0};
ros::Publisher g_pwm_publisher;
ros::Publisher g_vel_publisher;
ros::Publisher g_vel_body_publisher;
Eigen::Vector4f g_last_pwm = Eigen::Vector4f::Constant(0.18f);
bool g_pwm_initialized = false;
float g_pwm_slew_step = 0.04f;
bool g_stage1_only = true;
bool g_auto_path_on_init = true;
int g_stage1_waypoint_count = 120;
bool g_stage1_path_is_provisional = false;
Eigen::Matrix4d Tw0, Twb_last;
Eigen::Vector3d Pwend;
int next_goal_index = 0;
bool get_init_pose = false;
bool get_end_goal = false;
int trigger_port = 1;
int cb_cnt = 0;
bool istakeoff = false;
void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void gt_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void timeCB(const ros::TimerEvent& event);
