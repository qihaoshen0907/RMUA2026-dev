#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "airsim_ros/Takeoff.h"
#include "airsim_ros/RotorPWM.h"

#include "rpg_mpc/mpc_controller.h"
#include "rpg_mpc/mpc_params.h"

#include "quadrotor_common/quad_state_estimate.h"
#include "quadrotor_common/trajectory.h"
#include "quadrotor_common/control_command.h"

namespace
{
ros::Publisher g_pwm_pub;
ros::ServiceClient g_takeoff_client;

std::unique_ptr<rpg_mpc::MpcController<double>> g_mpc;
rpg_mpc::MpcParams<double> g_mpc_params;

bool g_has_odom = false;
bool g_takeoff_done = false;
bool g_control_enabled = false;

ros::Time g_takeoff_time;
nav_msgs::Odometry g_last_odom;

Eigen::Vector3d g_ref_origin_pos(0.0, 0.0, 0.0);
bool g_ref_origin_set = false;
ros::Time g_control_start_time;

std::vector<Eigen::Vector3d> g_path_offsets;
std::size_t g_target_waypoint_idx = 0;

// RMUA AirSim RotorPWM is treated as a normalized motor command here.
// These values are intentionally low until the hover PWM is identified.
constexpr double kHoverPwm = 0.185;
constexpr double kAltPwmGain = 0.010;
constexpr double kAltVelPwmGain = 0.018;
constexpr double kMinPwm = 0.13;
constexpr double kMaxPwm = 0.26;
constexpr double kForwardPitchTargetGain = 0.030;
constexpr double kMaxForwardPitchTarget = 0.080;
constexpr double kPitchAnglePwmGain = 0.010;
constexpr double kPitchRatePwmGain = 0.008;
constexpr double kMaxPitchPwm = 0.018;
constexpr double kLateralRollTargetGain = 0.025;
constexpr double kMaxLateralRollTarget = 0.080;
constexpr double kRollAnglePwmGain = 0.010;
constexpr double kRollRatePwmGain = 0.008;
constexpr double kMaxRollPwm = 0.018;
constexpr double kYawAnglePwmGain = 0.0;
constexpr double kYawRatePwmGain = 0.0;
constexpr double kMaxYawPwm = 0.0;
constexpr double kForwardPitchDelaySec = 0.5;
constexpr double kForwardPitchRampSec = 2.0;
constexpr double kSingleMotorMaxThrust = 12.538338804;

// Reference settings.
constexpr double kTakeoffSettleSec = 3.0;
constexpr double kForwardSpeed = 0.25;
constexpr double kForwardDistance = 3.0;
constexpr double kForwardLookahead = 0.8;
constexpr double kWaypointSpacing = 0.8;
constexpr double kWaypointSwitchMargin = 0.1;
constexpr double kControlRate = 100.0;
constexpr double kPi = 3.14159265358979323846;

double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}

double wrapAngle(double angle)
{
  while (angle > kPi)
  {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi)
  {
    angle += 2.0 * kPi;
  }
  return angle;
}

Eigen::Vector3d quaternionToRpy(const Eigen::Quaterniond& q)
{
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::asin(clamp(-R(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

bool loadPathOffsets(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open())
  {
    ROS_WARN("Failed to open trajectory file: %s", path.c_str());
    return false;
  }

  std::vector<Eigen::Vector3d> points;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  while (file >> x >> y >> z)
  {
    points.emplace_back(x, y, z);
  }

  if (points.size() < 2)
  {
    ROS_WARN(
        "Trajectory file has too few xyz triples: %s", path.c_str());
    return false;
  }

  g_path_offsets.clear();
  const Eigen::Vector3d first = points.front();
  double last_x = 0.0;
  g_path_offsets.emplace_back(0.0, 0.0, 0.0);

  for (const Eigen::Vector3d& point : points)
  {
    Eigen::Vector3d offset = point - first;
    offset.z() = 0.0;

    // Use x as the primary progress coordinate and keep points sparse enough
    // that crossing one target cleanly switches to the next.
    if (offset.x() > last_x + kWaypointSpacing)
    {
      g_path_offsets.push_back(offset);
      last_x = offset.x();
    }
  }

  ROS_INFO(
      "Loaded %zu forward waypoints from %s",
      g_path_offsets.size(),
      path.c_str());
  return g_path_offsets.size() >= 2;
}

quadrotor_common::QuadStateEstimate odomToQuadState(
    const nav_msgs::Odometry& odom)
{
  quadrotor_common::QuadStateEstimate state;

  state.timestamp = odom.header.stamp;

  state.position = Eigen::Vector3d(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);

  state.orientation = Eigen::Quaterniond(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z);
  state.orientation.normalize();

  state.velocity = Eigen::Vector3d(
      odom.twist.twist.linear.x,
      odom.twist.twist.linear.y,
      odom.twist.twist.linear.z);

  state.bodyrates = Eigen::Vector3d(
      odom.twist.twist.angular.x,
      odom.twist.twist.angular.y,
      odom.twist.twist.angular.z);

  return state;
}

quadrotor_common::Trajectory makeForwardReference(
    const Eigen::Vector3d& origin_pos,
    const Eigen::Vector3d& current_pos,
    const ros::Duration& elapsed)
{
  quadrotor_common::Trajectory traj;
  traj.trajectory_type = quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  Eigen::Vector3d target_pos = origin_pos;
  double target_speed = 0.0;

  if (!g_path_offsets.empty())
  {
    while (g_target_waypoint_idx + 1 < g_path_offsets.size() &&
           current_pos.x() >=
               origin_pos.x() + g_path_offsets[g_target_waypoint_idx].x() -
                   kWaypointSwitchMargin)
    {
      ++g_target_waypoint_idx;
    }

    target_pos = origin_pos + g_path_offsets[g_target_waypoint_idx];
    target_pos.z() = origin_pos.z();
    target_speed = g_target_waypoint_idx + 1 < g_path_offsets.size()
                       ? kForwardSpeed
                       : 0.0;
  }
  else
  {
    const double t = std::max(0.0, elapsed.toSec());
    const double time_progress = kForwardSpeed * t;
    const double current_progress =
        std::max(0.0, current_pos.x() - origin_pos.x());
    const double lookahead_progress = current_progress + kForwardLookahead;
    const double x_forward = std::min(
        kForwardDistance,
        std::max(time_progress, lookahead_progress));

    target_pos = origin_pos + Eigen::Vector3d(x_forward, 0.0, 0.0);
    target_speed = x_forward < kForwardDistance ? kForwardSpeed : 0.0;
  }

  quadrotor_common::TrajectoryPoint point;
  point.time_from_start = ros::Duration(0.0);

  point.position = target_pos;
  point.velocity = Eigen::Vector3d(target_speed, 0.0, 0.0);
  point.acceleration = Eigen::Vector3d::Zero();

  point.orientation = Eigen::Quaterniond::Identity();
  point.bodyrates = Eigen::Vector3d::Zero();
  point.heading = 0.0;
  point.heading_rate = 0.0;

  traj.points.push_back(point);
  return traj;
}

airsim_ros::RotorPWM commandToPwm(
    const quadrotor_common::QuadStateEstimate& state,
    const Eigen::Vector3d& target_pos,
    const ros::Duration& elapsed_since_control,
    double* base_pwm_out,
    double* target_roll_out,
    double* target_pitch_out,
    double* target_yaw_out,
    double* roll_pwm_out,
    double* pitch_pwm_out,
    double* yaw_pwm_out,
    Eigen::Vector4d* motor_force_out)
{
  // In the current odometry frame, falling after takeoff makes z increase.
  // Increase PWM when current z is larger than the hold target z.
  const double z_error_down = state.position.z() - target_pos.z();
  const double z_velocity_down = state.velocity.z();
  const double base_pwm = clamp(
      kHoverPwm + kAltPwmGain * z_error_down + kAltVelPwmGain * z_velocity_down,
      kMinPwm,
      kMaxPwm);
  if (base_pwm_out != nullptr)
  {
    *base_pwm_out = base_pwm;
  }

  const double pitch_ramp = clamp(
      (elapsed_since_control.toSec() - kForwardPitchDelaySec) /
          kForwardPitchRampSec,
      0.0,
      1.0);
  const Eigen::Vector3d rpy = quaternionToRpy(state.orientation);

  const double x_error = std::max(0.0, target_pos.x() - state.position.x());
  const double target_pitch = -pitch_ramp * clamp(
      kForwardPitchTargetGain * x_error,
      -kMaxForwardPitchTarget,
      kMaxForwardPitchTarget);
  if (target_pitch_out != nullptr)
  {
    *target_pitch_out = target_pitch;
  }

  const double y_error = target_pos.y() - state.position.y();
  const double target_roll = pitch_ramp * clamp(
      kLateralRollTargetGain * y_error,
      -kMaxLateralRollTarget,
      kMaxLateralRollTarget);
  if (target_roll_out != nullptr)
  {
    *target_roll_out = target_roll;
  }

  const double target_yaw = std::atan2(
      target_pos.y() - state.position.y(),
      std::max(0.1, target_pos.x() - state.position.x()));
  if (target_yaw_out != nullptr)
  {
    *target_yaw_out = target_yaw;
  }

  const double roll_error = target_roll - rpy.x();
  const double roll_pwm = clamp(
      kRollAnglePwmGain * roll_error - kRollRatePwmGain * state.bodyrates.x(),
      -kMaxRollPwm,
      kMaxRollPwm);
  if (roll_pwm_out != nullptr)
  {
    *roll_pwm_out = roll_pwm;
  }

  const double pitch_error = target_pitch - rpy.y();
  const double pitch_pwm = clamp(
      -kPitchAnglePwmGain * pitch_error +
          kPitchRatePwmGain * state.bodyrates.y(),
      -kMaxPitchPwm,
      kMaxPitchPwm);
  if (pitch_pwm_out != nullptr)
  {
    *pitch_pwm_out = pitch_pwm;
  }

  const double yaw_error = wrapAngle(target_yaw - rpy.z());
  const double yaw_pwm = clamp(
      kYawAnglePwmGain * yaw_error - kYawRatePwmGain * state.bodyrates.z(),
      -kMaxYawPwm,
      kMaxYawPwm);
  if (yaw_pwm_out != nullptr)
  {
    *yaw_pwm_out = yaw_pwm;
  }

  airsim_ros::RotorPWM pwm;
  pwm.header.stamp = ros::Time::now();

  Eigen::Vector4d pwm_vec;
  pwm_vec[0] = clamp(
      base_pwm - pitch_pwm - roll_pwm - yaw_pwm,
      kMinPwm,
      kMaxPwm);
  pwm_vec[1] = clamp(
      base_pwm + pitch_pwm + roll_pwm - yaw_pwm,
      kMinPwm,
      kMaxPwm);
  pwm_vec[2] = clamp(
      base_pwm - pitch_pwm + roll_pwm + yaw_pwm,
      kMinPwm,
      kMaxPwm);
  pwm_vec[3] = clamp(
      base_pwm + pitch_pwm - roll_pwm + yaw_pwm,
      kMinPwm,
      kMaxPwm);

  if (motor_force_out != nullptr)
  {
    for (int i = 0; i < 4; ++i)
    {
      (*motor_force_out)[i] = pwm_vec[i] * pwm_vec[i] * kSingleMotorMaxThrust;
    }
  }

  // RMUA motor index:
  // PWM0 RF, PWM1 LR, PWM2 LF, PWM3 RR.
  pwm.rotorPWM0 = pwm_vec[0];
  pwm.rotorPWM1 = pwm_vec[1];
  pwm.rotorPWM2 = pwm_vec[2];
  pwm.rotorPWM3 = pwm_vec[3];

  return pwm;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  g_last_odom = *msg;
  g_has_odom = true;
}

bool callTakeoff()
{
  ROS_INFO("Calling takeoff service...");

  if (!g_takeoff_client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_ERROR("Takeoff service not available.");
    return false;
  }

  airsim_ros::Takeoff srv;
  srv.request.waitOnLastTask = true;

  if (!g_takeoff_client.call(srv))
  {
    ROS_ERROR("Takeoff service call failed.");
    return false;
  }

  if (!srv.response.success)
  {
    ROS_ERROR("Takeoff service returned false.");
    return false;
  }

  g_takeoff_time = ros::Time::now();
  g_takeoff_done = true;

  ROS_INFO("Takeoff success.");
  return true;
}

void controlTimerCallback(const ros::TimerEvent&)
{
  if (!g_has_odom || !g_takeoff_done)
  {
    return;
  }

  const ros::Duration since_takeoff = ros::Time::now() - g_takeoff_time;

  // Start forward motion almost immediately after the takeoff task completes
  // so the drone leaves the takeoff judging area.
  if (since_takeoff.toSec() < kTakeoffSettleSec)
  {
    return;
  }

  quadrotor_common::QuadStateEstimate state = odomToQuadState(g_last_odom);

  if (!g_control_enabled)
  {
    g_ref_origin_pos = state.position;
    g_ref_origin_set = true;
    g_target_waypoint_idx = g_path_offsets.size() > 1 ? 1 : 0;
    g_control_start_time = ros::Time::now();
    g_control_enabled = true;

    ROS_INFO(
        "MPC forward control enabled. Reference origin: [%.2f %.2f %.2f]",
        g_ref_origin_pos.x(),
        g_ref_origin_pos.y(),
        g_ref_origin_pos.z());
  }

  quadrotor_common::Trajectory reference =
      makeForwardReference(
          g_ref_origin_pos,
          state.position,
          ros::Time::now() - g_control_start_time);
  const ros::Duration elapsed_since_control =
      ros::Time::now() - g_control_start_time;
  const Eigen::Vector3d target_pos = reference.points.front().position;
  const Eigen::Vector3d pos_error = target_pos - state.position;
  const Eigen::Vector3d rpy = quaternionToRpy(state.orientation);

  quadrotor_common::ControlCommand cmd =
      g_mpc->run(state, reference, g_mpc_params);

  double base_pwm = 0.0;
  double target_roll = 0.0;
  double target_pitch = 0.0;
  double target_yaw = 0.0;
  double roll_pwm = 0.0;
  double pitch_pwm = 0.0;
  double yaw_pwm = 0.0;
  Eigen::Vector4d motor_force = Eigen::Vector4d::Zero();
  airsim_ros::RotorPWM pwm =
      commandToPwm(
          state,
          target_pos,
          elapsed_since_control,
          &base_pwm,
          &target_roll,
          &target_pitch,
          &target_yaw,
          &roll_pwm,
          &pitch_pwm,
          &yaw_pwm,
          &motor_force);
  g_pwm_pub.publish(pwm);

  ROS_INFO_THROTTLE(
      0.5,
      "MPC cmd: thrust=%.2f, rates=[%.2f %.2f %.2f], "
      "wp=%zu/%zu, pos=[%.2f %.2f %.2f], target=[%.2f %.2f %.2f], "
      "rpy=[%.2f %.2f %.2f], bodyrates=[%.2f %.2f %.2f], "
      "err=[%.2f %.2f %.2f], base_pwm=%.2f, "
      "target_rpy=[%.3f %.3f %.3f], mix_pwm=[%.3f %.3f %.3f], "
      "pwm=[%.2f %.2f %.2f %.2f], "
      "force=[%.3f %.3f %.3f %.3f]",
      cmd.collective_thrust,
      cmd.bodyrates.x(),
      cmd.bodyrates.y(),
      cmd.bodyrates.z(),
      g_target_waypoint_idx,
      g_path_offsets.size(),
      state.position.x(),
      state.position.y(),
      state.position.z(),
      target_pos.x(),
      target_pos.y(),
      target_pos.z(),
      rpy.x(),
      rpy.y(),
      rpy.z(),
      state.bodyrates.x(),
      state.bodyrates.y(),
      state.bodyrates.z(),
      pos_error.x(),
      pos_error.y(),
      pos_error.z(),
      base_pwm,
      target_roll,
      target_pitch,
      target_yaw,
      roll_pwm,
      pitch_pwm,
      yaw_pwm,
      pwm.rotorPWM0,
      pwm.rotorPWM1,
      pwm.rotorPWM2,
      pwm.rotorPWM3,
      motor_force[0],
      motor_force[1],
      motor_force[2],
      motor_force[3]);
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_test");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  g_pwm_pub = nh.advertise<airsim_ros::RotorPWM>(
      "/airsim_node/drone_1/rotor_pwm_cmd", 1);

  g_takeoff_client = nh.serviceClient<airsim_ros::Takeoff>(
      "/airsim_node/drone_1/takeoff");

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/eskf_odom", 1, odomCallback);

  std::string trajectory_file;
  pnh.param<std::string>(
      "trajectory_file",
      trajectory_file,
      "/home/bai/RUMA_mpc111/smoothed_63.txt");
  loadPathOffsets(trajectory_file);

  if (!g_mpc_params.loadParameters(pnh))
  {
    ROS_ERROR("Failed to load MPC parameters.");
    return 1;
  }

  ROS_INFO(
      "controller_test RMUA debug build: hover_pwm=%.3f, alt_gain=%.3f, "
      "alt_vel_gain=%.3f, pwm_limit=[%.3f %.3f], "
      "pitch_target_gain=%.3f, max_target_pitch=%.3f, "
      "pitch_angle_gain=%.3f, pitch_rate_gain=%.3f, max_pitch_pwm=%.3f, "
      "roll_target_gain=%.3f, max_target_roll=%.3f, "
      "roll_angle_gain=%.3f, roll_rate_gain=%.3f, max_roll_pwm=%.3f, "
      "yaw_angle_gain=%.3f, yaw_rate_gain=%.3f, max_yaw_pwm=%.3f, "
      "pitch_delay=%.1f, pitch_ramp=%.1f, takeoff_settle=%.1f",
      kHoverPwm,
      kAltPwmGain,
      kAltVelPwmGain,
      kMinPwm,
      kMaxPwm,
      kForwardPitchTargetGain,
      kMaxForwardPitchTarget,
      kPitchAnglePwmGain,
      kPitchRatePwmGain,
      kMaxPitchPwm,
      kLateralRollTargetGain,
      kMaxLateralRollTarget,
      kRollAnglePwmGain,
      kRollRatePwmGain,
      kMaxRollPwm,
      kYawAnglePwmGain,
      kYawRatePwmGain,
      kMaxYawPwm,
      kForwardPitchDelaySec,
      kForwardPitchRampSec,
      kTakeoffSettleSec);

  g_mpc.reset(new rpg_mpc::MpcController<double>(
      nh, pnh, "mpc/predicted_trajectory"));

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  if (!callTakeoff())
  {
    return 1;
  }

  ros::Timer control_timer = nh.createTimer(
      ros::Duration(1.0 / kControlRate),
      controlTimerCallback);

  ros::spin();
  return 0;
}
