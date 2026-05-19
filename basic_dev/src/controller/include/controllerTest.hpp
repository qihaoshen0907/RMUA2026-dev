#pragma once
#include <ros/ros.h>
#include "airsim_ros/RotorPWM.h"
#include "airsim_ros/Takeoff.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "mpc_controller.hpp"

ros::ServiceClient g_takeoff_client;
std::vector<std::vector<Eigen::Vector3d>> globalPaths;
std::vector<Eigen::Vector3d> globalPath;
ros::Publisher g_pwm_publisher;
Eigen::Matrix4d Tw0, Twb_last;
Eigen::Vector3d Pwend;
std::unique_ptr<QuadrotorLinearMPC> g_mpc_controller;

int next_goal_index;
bool get_init_pose, get_end_goal;
int trigger_port = 1;
bool istakeoff = false;
bool g_takeoff_sent = false;
ros::Time g_takeoff_time;
std::string g_spline_file_path;
void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);