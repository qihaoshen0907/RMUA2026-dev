#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
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
#include "controller_defaults.hpp"

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
struct ReferenceWaypoint
{
  Eigen::Vector3d pos_offset = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};
std::vector<ReferenceWaypoint> g_reference_waypoints;

double g_mass = 0.9;
double g_arm_length = 0.18;
double g_Ixx = 0.0046890742;
double g_Iyy = 0.0069312;
double g_Izz = 0.010421166;
double g_Ct = 0.00036771704516278653;
double g_Cq = 4.888486266072161e-06;
double g_Fmax_per_rotor = 12.538338804;
double g_rate_kp_x = 4.0;
double g_rate_kp_y = 4.0;
double g_rate_kp_z = 2.0;
double g_rate_kd_x = 0.15;
double g_rate_kd_y = 0.15;
double g_rate_kd_z = 0.08;
double g_min_pwm = 0.05;
double g_max_pwm = 1.0;
double g_min_thrust_acc = 9.81;
double g_max_thrust_acc = 20.0;
double g_ref_vel_max = 1.2;
bool g_invert_odom_z = true;

// Reference settings.
constexpr double kTakeoffSettleSec = 3.0;
constexpr double kForwardSpeed = 0.25;
constexpr double kForwardDistance = 3.0;
constexpr double kForwardLookahead = 0.8;
constexpr double kWaypointSpacing = 0.8;
constexpr double kWaypointSwitchMargin = 0.1;
constexpr int kHorizonPoints = 21;
constexpr double kMpcDt = 0.1;
constexpr double kTorqueLimit = 2.0;
constexpr double kBodyrateErrDotLimit = 12.0;
constexpr double kPostEnableZHoldSec = 2.0;
constexpr double kControlRate = 100.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kStablePosErrNormMax = 0.8;
constexpr double kStableZErrMax = 0.35;
constexpr double kStableVelErrNormMax = 0.8;
constexpr double kStableTiltDegMax = 25.0;
constexpr double kStableRateErrNormMax = 1.2;
constexpr double kStableOdomAgeMaxSec = 0.08;

Eigen::Vector3d g_omega_err_prev = Eigen::Vector3d::Zero();
ros::Time g_omega_err_prev_time;
int g_thrust_at_min_count = 0;

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

Eigen::Vector3d clampNorm(const Eigen::Vector3d& v, double max_norm)
{
  if (max_norm <= 0.0)
  {
    return Eigen::Vector3d::Zero();
  }
  const double n = v.norm();
  if (n <= max_norm || n < 1e-9)
  {
    return v;
  }
  return v * (max_norm / n);
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

  std::vector<ReferenceWaypoint> waypoints;
  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    std::istringstream iss(line);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    if (!(iss >> x >> y >> z >> vx >> vy >> vz))
    {
      continue;
    }
    ReferenceWaypoint wp;
    wp.pos_offset = Eigen::Vector3d(x, y, z);
    wp.velocity = clampNorm(Eigen::Vector3d(vx, vy, vz), g_ref_vel_max);
    waypoints.push_back(wp);
  }

  if (waypoints.size() < 2)
  {
    ROS_WARN(
        "Trajectory file has too few xyz+vxvyvz samples: %s", path.c_str());
    return false;
  }

  g_reference_waypoints.clear();
  const Eigen::Vector3d first = waypoints.front().pos_offset;
  double last_x = 0.0;
  ReferenceWaypoint first_wp;
  first_wp.pos_offset = Eigen::Vector3d::Zero();
  first_wp.velocity = waypoints.front().velocity;
  g_reference_waypoints.push_back(first_wp);

  for (const ReferenceWaypoint& wp : waypoints)
  {
    ReferenceWaypoint offset_wp = wp;
    offset_wp.pos_offset = wp.pos_offset - first;

    // Use x as the primary progress coordinate and keep points sparse enough
    // that crossing one target cleanly switches to the next.
    if (offset_wp.pos_offset.x() > last_x + kWaypointSpacing)
    {
      g_reference_waypoints.push_back(offset_wp);
      last_x = offset_wp.pos_offset.x();
    }
  }

  ROS_INFO(
      "Loaded %zu xyz+vxvyvz waypoints from %s",
      g_reference_waypoints.size(),
      path.c_str());
  return g_reference_waypoints.size() >= 2;
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

  if (g_invert_odom_z)
  {
    state.position.z() = -state.position.z();
    state.velocity.z() = -state.velocity.z();
  }

  state.bodyrates = Eigen::Vector3d(
      odom.twist.twist.angular.x,
      odom.twist.twist.angular.y,
      odom.twist.twist.angular.z);

  return state;
}

struct PathSample
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

PathSample samplePathAtTime(
    const Eigen::Vector3d& origin_pos,
    double t_future,
    std::size_t base_wp_idx)
{
  PathSample result;

  if (g_reference_waypoints.empty())
  {
    result.position = origin_pos +
        Eigen::Vector3d(kForwardSpeed * t_future, 0.0, 0.0);
    result.velocity = Eigen::Vector3d(kForwardSpeed, 0.0, 0.0);
    return result;
  }

  Eigen::Vector3d base_world =
      origin_pos + g_reference_waypoints[base_wp_idx].pos_offset;
  const double target_arc_length = std::max(0.0, kForwardSpeed * t_future);

  if (target_arc_length < 1e-6)
  {
    result.position = base_world;
    result.velocity = g_reference_waypoints[base_wp_idx].velocity;
    return result;
  }

  Eigen::Vector3d prev = base_world;
  double accumulated = 0.0;
  for (std::size_t i = base_wp_idx + 1; i < g_reference_waypoints.size(); ++i)
  {
    const Eigen::Vector3d next = origin_pos + g_reference_waypoints[i].pos_offset;
    const Eigen::Vector3d seg = next - prev;
    const double seg_len = seg.norm();
    if (seg_len < 1e-6)
    {
      prev = next;
      continue;
    }
    if (accumulated + seg_len >= target_arc_length)
    {
      const double alpha = (target_arc_length - accumulated) / seg_len;
      result.position = prev + alpha * seg;
      result.velocity = seg.normalized() * kForwardSpeed;
      return result;
    }
    accumulated += seg_len;
    prev = next;
  }

  result.position = prev;
  result.velocity = Eigen::Vector3d::Zero();
  return result;
}

quadrotor_common::Trajectory makeForwardReference(
    const Eigen::Vector3d& origin_pos,
    const Eigen::Vector3d& current_pos,
    const ros::Duration& elapsed)
{
  quadrotor_common::Trajectory traj;
  traj.trajectory_type = quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  if (!g_reference_waypoints.empty())
  {
    while (g_target_waypoint_idx + 1 < g_reference_waypoints.size() &&
           current_pos.x() >=
               origin_pos.x() + g_reference_waypoints[g_target_waypoint_idx].pos_offset.x() -
                   kWaypointSwitchMargin)
    {
      ++g_target_waypoint_idx;
    }

  }

  for (int i = 0; i < kHorizonPoints; ++i)
  {
    const double t = i * kMpcDt;
    const PathSample sample = samplePathAtTime(
        origin_pos, t, g_target_waypoint_idx);

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(t);
    point.position = sample.position;
    point.velocity = sample.velocity;
    point.acceleration = Eigen::Vector3d::Zero();
    point.orientation = Eigen::Quaterniond::Identity();
    point.bodyrates = Eigen::Vector3d::Zero();
    point.heading = sample.velocity.norm() > 0.1
        ? std::atan2(sample.velocity.y(), sample.velocity.x())
        : 0.0;
    point.heading_rate = 0.0;
    traj.points.push_back(point);
  }
  return traj;
}

airsim_ros::RotorPWM mpcCommandToPwm(
    const quadrotor_common::QuadStateEstimate& state,
    const quadrotor_common::ControlCommand& cmd,
    double* total_thrust_out,
    Eigen::Vector3d* desired_bodyrates_out,
    Eigen::Vector3d* bodyrate_error_out,
    Eigen::Vector3d* bodyrate_error_dot_out,
    Eigen::Vector3d* desired_torque_out,
    Eigen::Vector4d* motor_force_out)
{
  const double total_thrust = g_mass *
      clamp(cmd.collective_thrust, g_min_thrust_acc, g_max_thrust_acc);
  if (total_thrust_out != nullptr)
  {
    *total_thrust_out = total_thrust;
  }

  const Eigen::Vector3d desired_bodyrates = cmd.bodyrates;
  if (desired_bodyrates_out != nullptr)
  {
    *desired_bodyrates_out = desired_bodyrates;
  }

  const Eigen::Vector3d bodyrate_error = desired_bodyrates - state.bodyrates;
  if (bodyrate_error_out != nullptr)
  {
    *bodyrate_error_out = bodyrate_error;
  }

  double dt = (state.timestamp - g_omega_err_prev_time).toSec();
  dt = clamp(dt, 0.001, 0.05);
  Eigen::Vector3d bodyrate_error_dot = Eigen::Vector3d::Zero();
  if (!g_omega_err_prev_time.isZero())
  {
    bodyrate_error_dot = (bodyrate_error - g_omega_err_prev) / dt;
  }
  bodyrate_error_dot.x() = clamp(
      bodyrate_error_dot.x(), -kBodyrateErrDotLimit, kBodyrateErrDotLimit);
  bodyrate_error_dot.y() = clamp(
      bodyrate_error_dot.y(), -kBodyrateErrDotLimit, kBodyrateErrDotLimit);
  bodyrate_error_dot.z() = clamp(
      bodyrate_error_dot.z(), -kBodyrateErrDotLimit, kBodyrateErrDotLimit);
  g_omega_err_prev = bodyrate_error;
  g_omega_err_prev_time = state.timestamp;
  if (bodyrate_error_dot_out != nullptr)
  {
    *bodyrate_error_dot_out = bodyrate_error_dot;
  }

  const Eigen::Vector3d inertia(g_Ixx, g_Iyy, g_Izz);
  const Eigen::Vector3d kp(g_rate_kp_x, g_rate_kp_y, g_rate_kp_z);
  const Eigen::Vector3d kd(g_rate_kd_x, g_rate_kd_y, g_rate_kd_z);
  const Eigen::Vector3d desired_torque_raw = inertia.cwiseProduct(
      kp.cwiseProduct(bodyrate_error) + kd.cwiseProduct(bodyrate_error_dot));
  const Eigen::Vector3d desired_torque(
      clamp(desired_torque_raw.x(), -kTorqueLimit, kTorqueLimit),
      clamp(desired_torque_raw.y(), -kTorqueLimit, kTorqueLimit),
      clamp(desired_torque_raw.z(), -kTorqueLimit, kTorqueLimit));
  if (desired_torque_out != nullptr)
  {
    *desired_torque_out = desired_torque;
  }

  airsim_ros::RotorPWM pwm;
  pwm.header.stamp = ros::Time::now();

  const double half_diag = g_arm_length / std::sqrt(2.0);
  const double yaw_coeff = std::max(1e-9, std::abs(g_Cq / g_Ct));

  Eigen::Matrix4d A;
  A << 1.0, 1.0, 1.0, 1.0,
      -half_diag, half_diag, half_diag, -half_diag,
      -half_diag, half_diag, -half_diag, half_diag,
      -yaw_coeff, -yaw_coeff, yaw_coeff, yaw_coeff;

  const Eigen::Vector4d wrench(
      total_thrust,
      desired_torque.x(),
      desired_torque.y(),
      desired_torque.z());
  Eigen::Vector4d motor_force =
      A.fullPivLu().solve(wrench);
  for (int i = 0; i < 4; ++i)
  {
    motor_force[i] = clamp(motor_force[i], 0.0, g_Fmax_per_rotor);
  }

  if (motor_force_out != nullptr)
  {
    *motor_force_out = motor_force;
  }

  Eigen::Vector4d pwm_vec;
  for (int i = 0; i < 4; ++i)
  {
    const double normalized = motor_force[i] / g_Fmax_per_rotor;
    pwm_vec[i] = clamp(normalized, g_min_pwm, g_max_pwm);
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
    g_target_waypoint_idx = g_reference_waypoints.size() > 1 ? 1 : 0;
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
  const double since_control_enable =
      (ros::Time::now() - g_control_start_time).toSec();
  if (since_control_enable < kPostEnableZHoldSec)
  {
    for (auto& point : reference.points)
    {
      point.position.z() = g_ref_origin_pos.z();
      point.velocity.z() = 0.0;
      point.acceleration.z() = 0.0;
    }
  }
  const quadrotor_common::TrajectoryPoint& ref_point = reference.points.front();
  const Eigen::Vector3d target_pos = ref_point.position;
  const Eigen::Vector3d pos_error = target_pos - state.position;
  const Eigen::Vector3d rpy = quaternionToRpy(state.orientation);
  const Eigen::Vector3d target_rpy = quaternionToRpy(ref_point.orientation);
  const Eigen::Vector3d vel_error = ref_point.velocity - state.velocity;

  quadrotor_common::ControlCommand cmd =
      g_mpc->run(state, reference, g_mpc_params);
  const bool thrust_at_min =
      cmd.collective_thrust <= g_min_thrust_acc + 1e-3;
  if (thrust_at_min)
  {
    ++g_thrust_at_min_count;
  }
  else
  {
    g_thrust_at_min_count = 0;
  }
  ROS_WARN_THROTTLE(
      1.0,
      "THRUST_DIAG cmd=%.2f limit=[%.2f %.2f] at_min=%d streak=%d",
      cmd.collective_thrust,
      g_min_thrust_acc,
      g_max_thrust_acc,
      static_cast<int>(thrust_at_min),
      g_thrust_at_min_count);

  double total_thrust = 0.0;
  Eigen::Vector3d desired_bodyrates = Eigen::Vector3d::Zero();
  Eigen::Vector3d bodyrate_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d bodyrate_error_dot = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_torque = Eigen::Vector3d::Zero();
  Eigen::Vector4d motor_force = Eigen::Vector4d::Zero();
  airsim_ros::RotorPWM pwm =
      mpcCommandToPwm(
          state,
          cmd,
          &total_thrust,
          &desired_bodyrates,
          &bodyrate_error,
          &bodyrate_error_dot,
          &desired_torque,
          &motor_force);
  g_pwm_pub.publish(pwm);

  ROS_INFO_THROTTLE(
      0.5,
      "MPC cmd: thrust=%.2f, rates=[%.2f %.2f %.2f], "
      "wp=%zu/%zu, pos=[%.2f %.2f %.2f], target=[%.2f %.2f %.2f], "
      "target_rpy=[%.2f %.2f %.2f], rpy=[%.2f %.2f %.2f], bodyrates=[%.2f %.2f %.2f], "
      "reference={pos=[%.2f %.2f %.2f], vel=[%.2f %.2f %.2f], acc=[%.2f %.2f %.2f], heading=%.2f, heading_rate=%.2f}, "
      "err=[%.2f %.2f %.2f], "
      "T=%.2f, rate_err=[%.3f %.3f %.3f], rate_err_dot=[%.3f %.3f %.3f], tau=[%.3f %.3f %.3f], "
      "pwm=[%.2f %.2f %.2f %.2f], "
      "force=[%.3f %.3f %.3f %.3f], motor={RF:%.2f/%.3f, LR:%.2f/%.3f, LF:%.2f/%.3f, RR:%.2f/%.3f}",
      cmd.collective_thrust,
      cmd.bodyrates.x(),
      cmd.bodyrates.y(),
      cmd.bodyrates.z(),
      g_target_waypoint_idx,
      g_reference_waypoints.size(),
      state.position.x(),
      state.position.y(),
      state.position.z(),
      target_pos.x(),
      target_pos.y(),
      target_pos.z(),
      target_rpy.x(),
      target_rpy.y(),
      target_rpy.z(),
      rpy.x(),
      rpy.y(),
      rpy.z(),
      state.bodyrates.x(),
      state.bodyrates.y(),
      state.bodyrates.z(),
      ref_point.position.x(),
      ref_point.position.y(),
      ref_point.position.z(),
      ref_point.velocity.x(),
      ref_point.velocity.y(),
      ref_point.velocity.z(),
      ref_point.acceleration.x(),
      ref_point.acceleration.y(),
      ref_point.acceleration.z(),
      ref_point.heading,
      ref_point.heading_rate,
      pos_error.x(),
      pos_error.y(),
      pos_error.z(),
      total_thrust,
      bodyrate_error.x(),
      bodyrate_error.y(),
      bodyrate_error.z(),
      bodyrate_error_dot.x(),
      bodyrate_error_dot.y(),
      bodyrate_error_dot.z(),
      desired_torque.x(),
      desired_torque.y(),
      desired_torque.z(),
      pwm.rotorPWM0,
      pwm.rotorPWM1,
      pwm.rotorPWM2,
      pwm.rotorPWM3,
      motor_force[0],
      motor_force[1],
      motor_force[2],
      motor_force[3],
      pwm.rotorPWM0,
      motor_force[0],
      pwm.rotorPWM1,
      motor_force[1],
      pwm.rotorPWM2,
      motor_force[2],
      pwm.rotorPWM3,
      motor_force[3]);

  const double now_sec = ros::Time::now().toSec();
  const double odom_age_sec = now_sec - g_last_odom.header.stamp.toSec();
  const double pos_err_norm = pos_error.norm();
  const double vel_err_norm = vel_error.norm();
  const double tilt_deg = std::sqrt(rpy.x() * rpy.x() + rpy.y() * rpy.y()) * 180.0 / kPi;
  const double yaw_err_deg = std::abs(wrapAngle(target_rpy.z() - rpy.z())) * 180.0 / kPi;
  const double rate_err_norm = bodyrate_error.norm();
  const double rate_err_dot_norm = bodyrate_error_dot.norm();
  const double torque_norm = desired_torque.norm();
  const double pwm_min = std::min(std::min(pwm.rotorPWM0, pwm.rotorPWM1),
                                  std::min(pwm.rotorPWM2, pwm.rotorPWM3));
  const double pwm_max = std::max(std::max(pwm.rotorPWM0, pwm.rotorPWM1),
                                  std::max(pwm.rotorPWM2, pwm.rotorPWM3));
  int sat_pwm_low = 0;
  int sat_pwm_high = 0;
  const double pwm_values[4] = {pwm.rotorPWM0, pwm.rotorPWM1, pwm.rotorPWM2, pwm.rotorPWM3};
  for (double pwm_i : pwm_values)
  {
    if (pwm_i <= g_min_pwm + 1e-6) ++sat_pwm_low;
    if (pwm_i >= g_max_pwm - 1e-6) ++sat_pwm_high;
  }
  const bool stable = (pos_err_norm < kStablePosErrNormMax) &&
                      (std::abs(pos_error.z()) < kStableZErrMax) &&
                      (vel_err_norm < kStableVelErrNormMax) &&
                      (tilt_deg < kStableTiltDegMax) &&
                      (rate_err_norm < kStableRateErrNormMax) &&
                      (odom_age_sec >= 0.0 && odom_age_sec < kStableOdomAgeMaxSec) &&
                      (sat_pwm_high == 0);

  ROS_INFO_THROTTLE(
      0.5,
      "FLIGHT_DIAG stable=%d "
      "odom_age=%.3f "
      "e_pos_norm=%.2f e_pos_z=%.2f e_vel_norm=%.2f "
      "tilt_deg=%.1f yaw_err_deg=%.1f "
      "e_rate_norm=%.2f e_rate_dot_norm=%.2f tau_norm=%.3f "
      "pwm_min=%.2f pwm_max=%.2f sat_low=%d sat_high=%d "
      "sim_pos=[%.2f %.2f %.2f] sim_vel=[%.2f %.2f %.2f] sim_rpy_deg=[%.1f %.1f %.1f]",
      static_cast<int>(stable),
      odom_age_sec,
      pos_err_norm,
      pos_error.z(),
      vel_err_norm,
      tilt_deg,
      yaw_err_deg,
      rate_err_norm,
      rate_err_dot_norm,
      torque_norm,
      pwm_min,
      pwm_max,
      sat_pwm_low,
      sat_pwm_high,
      state.position.x(),
      state.position.y(),
      state.position.z(),
      state.velocity.x(),
      state.velocity.y(),
      state.velocity.z(),
      rpy.x() * 180.0 / kPi,
      rpy.y() * 180.0 / kPi,
      rpy.z() * 180.0 / kPi);
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_test");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  auto setParamIfMissing = [&pnh](const std::string& name, const auto& value) {
    if (!pnh.hasParam(name))
    {
      pnh.setParam(name, value);
    }
  };
  const auto& d = controller_defaults::kDefaults;
  setParamIfMissing("trajectory_file", d.trajectory_file);
  setParamIfMissing("Q_pos_xy", d.Q_pos_xy);
  setParamIfMissing("Q_pos_z", d.Q_pos_z);
  setParamIfMissing("Q_attitude", d.Q_attitude);
  setParamIfMissing("Q_velocity", d.Q_velocity);
  setParamIfMissing("Q_perception", d.Q_perception);
  setParamIfMissing("R_thrust", d.R_thrust);
  setParamIfMissing("R_pitchroll", d.R_pitchroll);
  setParamIfMissing("R_yaw", d.R_yaw);
  setParamIfMissing("min_thrust", d.min_thrust);
  setParamIfMissing("max_thrust", d.max_thrust);
  setParamIfMissing("max_bodyrate_xy", d.max_bodyrate_xy);
  setParamIfMissing("max_bodyrate_z", d.max_bodyrate_z);
  setParamIfMissing("mass", d.mass);
  setParamIfMissing("arm_length", d.arm_length);
  setParamIfMissing("Ixx", d.Ixx);
  setParamIfMissing("Iyy", d.Iyy);
  setParamIfMissing("Izz", d.Izz);
  setParamIfMissing("Ct", d.Ct);
  setParamIfMissing("Cq", d.Cq);
  setParamIfMissing("Fmax_per_rotor", d.Fmax_per_rotor);
  setParamIfMissing("rate_kp_x", d.rate_kp_x);
  setParamIfMissing("rate_kp_y", d.rate_kp_y);
  setParamIfMissing("rate_kp_z", d.rate_kp_z);
  setParamIfMissing("rate_kd_x", d.rate_kd_x);
  setParamIfMissing("rate_kd_y", d.rate_kd_y);
  setParamIfMissing("rate_kd_z", d.rate_kd_z);
  setParamIfMissing("min_pwm", d.min_pwm);
  setParamIfMissing("max_pwm", d.max_pwm);
  setParamIfMissing("ref_vel_max", d.ref_vel_max);
  setParamIfMissing("invert_odom_z", d.invert_odom_z);

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
      d.trajectory_file);

  if (!g_mpc_params.loadParameters(pnh))
  {
    ROS_ERROR("Failed to load MPC parameters.");
    return 1;
  }
  g_min_thrust_acc = g_mpc_params.min_thrust_;
  g_max_thrust_acc = g_mpc_params.max_thrust_;

  pnh.param("mass", g_mass, g_mass);
  pnh.param("arm_length", g_arm_length, g_arm_length);
  pnh.param("Ixx", g_Ixx, g_Ixx);
  pnh.param("Iyy", g_Iyy, g_Iyy);
  pnh.param("Izz", g_Izz, g_Izz);
  pnh.param("Ct", g_Ct, g_Ct);
  pnh.param("Cq", g_Cq, g_Cq);
  pnh.param("Fmax_per_rotor", g_Fmax_per_rotor, g_Fmax_per_rotor);
  pnh.param("rate_kp_x", g_rate_kp_x, g_rate_kp_x);
  pnh.param("rate_kp_y", g_rate_kp_y, g_rate_kp_y);
  pnh.param("rate_kp_z", g_rate_kp_z, g_rate_kp_z);
  pnh.param("rate_kd_x", g_rate_kd_x, g_rate_kd_x);
  pnh.param("rate_kd_y", g_rate_kd_y, g_rate_kd_y);
  pnh.param("rate_kd_z", g_rate_kd_z, g_rate_kd_z);
  pnh.param("min_pwm", g_min_pwm, g_min_pwm);
  pnh.param("max_pwm", g_max_pwm, g_max_pwm);
  pnh.param("ref_vel_max", g_ref_vel_max, g_ref_vel_max);
  pnh.param("invert_odom_z", g_invert_odom_z, g_invert_odom_z);
  pnh.param("min_thrust", g_min_thrust_acc, g_min_thrust_acc);
  pnh.param("max_thrust", g_max_thrust_acc, g_max_thrust_acc);

  // Load waypoints after runtime parameters are finalized (e.g. ref_vel_max).
  loadPathOffsets(trajectory_file);

  double q_pos_xy_log = d.Q_pos_xy;
  double q_pos_z_log = d.Q_pos_z;
  double q_attitude_log = d.Q_attitude;
  double q_velocity_log = d.Q_velocity;
  double r_thrust_log = d.R_thrust;
  double r_pitchroll_log = d.R_pitchroll;
  double r_yaw_log = d.R_yaw;
  double max_bodyrate_xy_log = d.max_bodyrate_xy;
  double max_bodyrate_z_log = d.max_bodyrate_z;
  pnh.param("Q_pos_xy", q_pos_xy_log, q_pos_xy_log);
  pnh.param("Q_pos_z", q_pos_z_log, q_pos_z_log);
  pnh.param("Q_attitude", q_attitude_log, q_attitude_log);
  pnh.param("Q_velocity", q_velocity_log, q_velocity_log);
  pnh.param("R_thrust", r_thrust_log, r_thrust_log);
  pnh.param("R_pitchroll", r_pitchroll_log, r_pitchroll_log);
  pnh.param("R_yaw", r_yaw_log, r_yaw_log);
  pnh.param("max_bodyrate_xy", max_bodyrate_xy_log, max_bodyrate_xy_log);
  pnh.param("max_bodyrate_z", max_bodyrate_z_log, max_bodyrate_z_log);

  ROS_INFO(
      "controller_test MPC->PWM build: mass=%.3f, arm=%.3f, "
      "I=[%.6f %.6f %.6f], Ct=%.9f, Cq=%.9f, Fmax=%.3f, "
      "rate_kp=[%.2f %.2f %.2f], rate_kd=[%.3f %.3f %.3f], "
      "thrust_acc=[%.2f %.2f], "
      "pwm_limit=[%.3f %.3f], ref_vel_max=%.2f, invert_odom_z=%d, horizon=%d, dt=%.2f, Q=[%.1f %.1f %.1f %.1f], R=[%.1f %.1f %.1f], "
      "max_bodyrate=[%.2f %.2f], err_dot_limit=%.1f, z_hold_after_enable=%.1f, takeoff_settle=%.1f",
      g_mass,
      g_arm_length,
      g_Ixx,
      g_Iyy,
      g_Izz,
      g_Ct,
      g_Cq,
      g_Fmax_per_rotor,
      g_rate_kp_x,
      g_rate_kp_y,
      g_rate_kp_z,
      g_rate_kd_x,
      g_rate_kd_y,
      g_rate_kd_z,
      g_min_thrust_acc,
      g_max_thrust_acc,
      g_min_pwm,
      g_max_pwm,
      g_ref_vel_max,
      static_cast<int>(g_invert_odom_z),
      kHorizonPoints,
      kMpcDt,
      q_pos_xy_log,
      q_pos_z_log,
      q_attitude_log,
      q_velocity_log,
      r_thrust_log,
      r_pitchroll_log,
      r_yaw_log,
      max_bodyrate_xy_log,
      max_bodyrate_z_log,
      kBodyrateErrDotLimit,
      kPostEnableZHoldSec,
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
