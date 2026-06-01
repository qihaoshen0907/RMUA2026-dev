#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "airsim_ros/Takeoff.h"
#include "airsim_ros/RotorPWM.h"
#include "airsim_ros/VelCmd.h"
#include <geometry_msgs/TwistStamped.h>

#include "path_spline.hpp"
#include "rpg_mpc/mpc_controller.h"
#include "rpg_mpc/mpc_params.h"

#include "quadrotor_common/quad_state_estimate.h"
#include "quadrotor_common/trajectory.h"
#include "quadrotor_common/control_command.h"
#include "controller_defaults.hpp"

namespace
{
ros::Publisher g_pwm_pub;
ros::Publisher g_vel_cmd_pub;
ros::Publisher g_ref_vel_pub;
ros::ServiceClient g_takeoff_client;

enum class ControlOutputMode
{
  kPwm,
  kVelocity
};

ControlOutputMode g_control_output_mode = ControlOutputMode::kVelocity;
std::string g_vel_cmd_topic = "/airsim_node/drone_1/vel_body_cmd";
int g_vel_mpc_node_index = 1;
double g_mpc_body_kp_pos_xy = 0.8;
double g_mpc_body_kp_pos_z = 0.6;
double g_mpc_body_kd_vel = 0.15;
double g_mpc_body_kp_yaw = 1.8;
bool g_vel_cmd_flip_body_y = true;
double g_body_cmd_max_vx = 10.0;
double g_body_cmd_max_vy = 4.0;
double g_body_cmd_max_vz = 2.0;
double g_body_cmd_max_yaw_rate = 1.5;
double g_body_cmd_max_acc_step = 1.0;
double g_cross_slowdown_m = 2.0;
double g_cross_progress_hold_m = 2.0;
double g_along_lag_lookahead_scale = 0.4;
double g_max_cross_track_m = 3.0;
bool g_vel_yaw_rate_in_deg = false;
bool g_align_path_yaw_to_body = true;
double g_path_yaw_offset = 0.0;
int g_vel_cmd_va = 1;

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

path_spline::PathSpline3D g_path_spline;
std::size_t g_nearest_wp_idx = 0;
double g_nearest_arc_s = 0.0;
double g_path_progress_s = 0.0;

double g_trajectory_dt = 0.1;
double g_cruise_speed = 1.2;
int g_waypoint_stride = 1;
int g_trajectory_smooth_window = 3;
double g_waypoint_max_segment_m = 5.0;
bool g_align_trajectory_to_start = true;
int g_reference_lookahead_points = 2;
std::size_t g_nearest_search_back = 60;
std::size_t g_nearest_search_ahead = 200;
double g_max_speed = 2.0;
double g_max_accel = 1.5;
double g_path_lookahead_m = 0.8;
double g_max_progress_ahead_m = 3.0;
bool g_odom_ned_to_enu = true;

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
// Reference settings.
constexpr double kTakeoffSettleSec = 3.0;
constexpr int kHorizonPoints = 21;
constexpr double kProgressBacktrackM = 0.3;
constexpr double kTorqueLimit = 2.0;
constexpr double kBodyrateErrDotLimit = 12.0;
constexpr double kPostEnableZHoldSec = 4.0;
constexpr double kPostEnableZRampSec = 3.0;
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

// NED world -> ENU world (paper Eq.2.1).
const Eigen::Matrix3d kRnedToEnu = (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                                                    1.0, 0.0, 0.0,
                                                    0.0, 0.0, -1.0).finished();
// AirSim body FRD -> MPC body FLU (paper Eq.2.2): Rx(pi).
const Eigen::Quaterniond kQfrdToFlu(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));

Eigen::Vector3d nedPositionToEnu(const Eigen::Vector3d& p_ned)
{
  return kRnedToEnu * p_ned;
}

Eigen::Vector3d nedVelocityToEnu(const Eigen::Vector3d& v_ned)
{
  return kRnedToEnu * v_ned;
}

// R_enu_flu_wb = R_enu_ned * R_ned_frd_wb * R_frd_flu
Eigen::Quaterniond nedAttitudeToEnuFlu(const Eigen::Quaterniond& q_ned_frd)
{
  const Eigen::Quaterniond q_R(kRnedToEnu);
  Eigen::Quaterniond q_enu_flu = q_R * q_ned_frd * kQfrdToFlu;
  q_enu_flu.normalize();
  return q_enu_flu;
}

Eigen::Vector3d frdBodyratesToFlu(const Eigen::Vector3d& omega_frd)
{
  return Eigen::Vector3d(omega_frd.x(), -omega_frd.y(), -omega_frd.z());
}

Eigen::Vector3d fluBodyVelToSimCmd(const Eigen::Vector3d& v_flu)
{
  Eigen::Vector3d v = v_flu;
  if (g_vel_cmd_flip_body_y)
  {
    v.y() = -v.y();
  }
  return v;
}

double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}

double normalizeAngle(double angle)
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

double wrapAngle(double angle)
{
  return normalizeAngle(angle);
}

Eigen::Vector3d worldVelToBodyVel(const Eigen::Vector3d& v_world, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector3d(
      c * v_world.x() + s * v_world.y(),
      -s * v_world.x() + c * v_world.y(),
      v_world.z());
}

bool isFiniteVector(const Eigen::Vector3d& v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) &&
      std::isfinite(v.z());
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

double getYawFromOdom(const nav_msgs::Odometry& odom)
{
  Eigen::Quaterniond q(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z);
  q.normalize();
  return quaternionToRpy(q).z();
}

bool parseNumericRow(const std::string& line, std::vector<double>* out)
{
  out->clear();
  if (line.empty() || line[0] == '#')
  {
    return false;
  }
  std::istringstream iss(line);
  double v = 0.0;
  while (iss >> v)
  {
    out->push_back(v);
  }
  return !out->empty();
}

bool fileExists(const std::string& path)
{
  std::ifstream file(path);
  return file.good();
}

std::string resolveTrajectoryPath(const std::string& path)
{
  if (path.empty() || path.front() == '/' || fileExists(path))
  {
    return path;
  }

  const std::string package_path = ros::package::getPath("controller_test");
  if (package_path.empty())
  {
    return path;
  }

  const std::string package_relative = package_path + "/" + path;
  if (fileExists(package_relative))
  {
    return package_relative;
  }

  return path;
}

// trajectory_for_mpc.txt formats:
//  A) Three rows: all x, then all y, then all z (same count per row).
//  B) Flat stream: x1 y1 z1 x2 y2 z2 ... (every three numbers = one waypoint).
//  C) Legacy: one waypoint per line "x y z" or "x y z vx vy vz".
bool loadTrajectorySpline(const std::string& path)
{
  const std::string resolved_path = resolveTrajectoryPath(path);
  std::ifstream file(resolved_path);
  if (!file.is_open())
  {
    ROS_WARN(
        "Failed to open trajectory file: %s (resolved: %s)",
        path.c_str(),
        resolved_path.c_str());
    return false;
  }

  std::vector<std::vector<double>> rows;
  std::vector<double> flat;
  std::string line;
  while (std::getline(file, line))
  {
    std::vector<double> row;
    if (!parseNumericRow(line, &row))
    {
      continue;
    }
    rows.push_back(row);
    flat.insert(flat.end(), row.begin(), row.end());
  }

  std::vector<Eigen::Vector3d> raw_points;
  const char* format_name = "unknown";

  std::size_t rows_with_xyz = 0;
  for (const std::vector<double>& row : rows)
  {
    if (row.size() >= 3)
    {
      ++rows_with_xyz;
    }
  }
  const bool mostly_xyz_per_line =
      rows.size() >= 2 &&
      rows_with_xyz >= static_cast<std::size_t>(0.9 * static_cast<double>(rows.size()));

  // Format A: exactly 3 lines — row1=all x, row2=all y, row3=all z (same count).
  if (rows.size() == 3 && rows[0].size() >= 2 &&
      rows[0].size() == rows[1].size() && rows[0].size() == rows[2].size())
  {
    format_name = "xyz_rows (line1=x, line2=y, line3=z)";
    raw_points.reserve(rows[0].size());
    for (std::size_t i = 0; i < rows[0].size(); ++i)
    {
      raw_points.emplace_back(rows[0][i], rows[1][i], rows[2][i]);
    }
  }
  else if (rows.size() == 3)
  {
    ROS_WARN(
        "Trajectory %s: three rows detected but x/y/z counts differ "
        "(%zu, %zu, %zu)",
        path.c_str(),
        rows[0].size(),
        rows[1].size(),
        rows[2].size());
    return false;
  }
  // Format B: each line is "x y z" (most common; do NOT use flat triplet merge).
  else if (mostly_xyz_per_line)
  {
    format_name = "xyz_per_line (each line: x y z)";
    raw_points.reserve(rows_with_xyz);
    for (const std::vector<double>& row : rows)
    {
      if (row.size() < 3)
      {
        continue;
      }
      raw_points.emplace_back(row[0], row[1], row[2]);
    }
  }
  // Format C: one line "x1 y1 z1 x2 y2 z2 ..."
  else if (rows.size() == 1 && flat.size() >= 6 && flat.size() % 3 == 0)
  {
    format_name = "xyz_triplets (flat stream, every 3 values)";
    raw_points.reserve(flat.size() / 3);
    for (std::size_t i = 0; i + 2 < flat.size(); i += 3)
    {
      raw_points.emplace_back(flat[i], flat[i + 1], flat[i + 2]);
    }
  }
  else
  {
    ROS_WARN(
        "Trajectory %s: unrecognized layout (%zu numeric rows)",
        path.c_str(),
        rows.size());
    return false;
  }

  if (raw_points.size() < 2)
  {
    ROS_WARN(
        "Trajectory file has too few xyz samples (%zu) in %s",
        raw_points.size(),
        path.c_str());
    return false;
  }

  if (g_align_trajectory_to_start)
  {
    const Eigen::Vector3d origin = raw_points.front();
    for (Eigen::Vector3d& p : raw_points)
    {
      p -= origin;
    }
  }

  const std::size_t file_waypoints = raw_points.size();
  if (!raw_points.empty())
  {
    const Eigen::Vector3d& p0 = raw_points.front();
    const Eigen::Vector3d& p1 = raw_points.size() > 1 ? raw_points[1] : p0;
    const Eigen::Vector3d& pn = raw_points.back();
    ROS_INFO(
        "Trajectory sample (aligned local frame): "
        "P0=[%.3f %.3f %.3f] P1=[%.3f %.3f %.3f] P_end=[%.3f %.3f %.3f]",
        p0.x(),
        p0.y(),
        p0.z(),
        p1.x(),
        p1.y(),
        p1.z(),
        pn.x(),
        pn.y(),
        pn.z());
  }
  if (g_waypoint_max_segment_m > 0.0)
  {
    raw_points = path_spline::densifyWaypointsByMaxSegment(
        raw_points, g_waypoint_max_segment_m);
  }

  const int stride = std::max(1, g_waypoint_stride);
  std::vector<Eigen::Vector3d> sparse_points;
  sparse_points.reserve(raw_points.size() / static_cast<std::size_t>(stride) + 1);
  for (std::size_t i = 0; i < raw_points.size(); i += static_cast<std::size_t>(stride))
  {
    sparse_points.push_back(raw_points[i]);
  }
  if (sparse_points.back() != raw_points.back())
  {
    sparse_points.push_back(raw_points.back());
  }

  const std::vector<Eigen::Vector3d> smoothed =
      path_spline::smoothWaypoints(sparse_points, g_trajectory_smooth_window);

  if (!g_path_spline.buildFromWaypoints(smoothed))
  {
    ROS_WARN("Failed to build arc-length spline from %s", path.c_str());
    return false;
  }

  g_nearest_wp_idx = 0;
  g_path_progress_s = 0.0;
  ROS_INFO(
      "Loaded spline path: format=%s, file_pts=%zu, after_densify=%zu, "
      "spline_pts=%zu, length=%.2f m from %s "
      "(stride=%d, max_seg=%.1f m, smooth_window=%d, align_start=%d, cruise=%.2f m/s)",
      format_name,
      file_waypoints,
      raw_points.size(),
      smoothed.size(),
      g_path_spline.totalLength(),
      resolved_path.c_str(),
      stride,
      g_waypoint_max_segment_m,
      g_trajectory_smooth_window,
      static_cast<int>(g_align_trajectory_to_start),
      g_cruise_speed);
  return true;
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

  if (g_odom_ned_to_enu)
  {
    state.position = nedPositionToEnu(state.position);
    state.velocity = nedVelocityToEnu(state.velocity);
    state.orientation = nedAttitudeToEnuFlu(state.orientation);
    state.bodyrates = frdBodyratesToFlu(state.bodyrates);
  }

  return state;
}

void rotateYawEnu(Eigen::Vector3d* v, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double x = v->x();
  const double y = v->y();
  v->x() = c * x - s * y;
  v->y() = s * x + c * y;
}

void updatePathProgress(const Eigen::Vector3d& current_pos)
{
  if (g_path_spline.numWaypoints() < 2)
  {
    return;
  }

  const Eigen::Vector3d local_pos = [&]() {
    Eigen::Vector3d p =
        g_align_trajectory_to_start && g_ref_origin_set
            ? current_pos - g_ref_origin_pos
            : current_pos;
    if (std::abs(g_path_yaw_offset) > 1e-6)
    {
      rotateYawEnu(&p, -g_path_yaw_offset);
    }
    return p;
  }();

  std::size_t search_back = g_nearest_search_back;
  std::size_t search_ahead = g_nearest_search_ahead;
  g_nearest_wp_idx = g_path_spline.nearestWaypointIndex(
      local_pos,
      g_nearest_wp_idx,
      search_back,
      search_ahead);

  double nearest_s = g_path_spline.closestArcLength(
      local_pos,
      g_nearest_wp_idx,
      search_back,
      search_ahead);

  const path_spline::Sample nearest_sample =
      g_path_spline.sampleAtArcLength(nearest_s, 0.0);
  const double cross_dist =
      (nearest_sample.position - local_pos).norm();
  if (cross_dist > g_max_cross_track_m)
  {
    search_back = std::min(g_path_spline.numWaypoints() - 1, std::size_t{400});
    search_ahead = std::min(g_path_spline.numWaypoints() - 1, std::size_t{400});
    g_nearest_wp_idx = g_path_spline.nearestWaypointIndex(
        local_pos, g_nearest_wp_idx, search_back, search_ahead);
    nearest_s = g_path_spline.closestArcLength(
        local_pos, g_nearest_wp_idx, search_back, search_ahead);
  }

  g_nearest_arc_s = nearest_s;
  double lookahead_s =
      static_cast<double>(std::max(1, g_reference_lookahead_points)) *
      g_path_lookahead_m;
  if (cross_dist > g_cross_slowdown_m)
  {
    lookahead_s *= g_along_lag_lookahead_scale;
  }

  // When far from path, do not run progress ahead of projection (avoids target jump).
  double desired_s = nearest_s + lookahead_s;
  if (cross_dist > g_cross_progress_hold_m)
  {
    desired_s = nearest_s;
  }
  else
  {
    desired_s = std::min(
        desired_s,
        nearest_s + std::max(0.5, g_max_progress_ahead_m));
  }
  g_path_progress_s = clamp(
      desired_s,
      std::max(nearest_s - 0.3, g_path_progress_s - kProgressBacktrackM),
      g_path_spline.totalLength());
}

path_spline::Sample sampleSplineWorld(
    const Eigen::Vector3d& origin_pos,
    double arc_s,
    double speed)
{
  path_spline::Sample local =
      g_path_spline.sampleAtArcLength(arc_s, speed);
  if (std::abs(g_path_yaw_offset) > 1e-6)
  {
    rotateYawEnu(&local.position, g_path_yaw_offset);
    rotateYawEnu(&local.velocity, g_path_yaw_offset);
    rotateYawEnu(&local.acceleration, g_path_yaw_offset);
  }
  if (g_align_trajectory_to_start)
  {
    local.position += origin_pos;
  }
  local.velocity = clampNorm(local.velocity, g_max_speed);
  local.acceleration = clampNorm(local.acceleration, g_max_accel);
  return local;
}

quadrotor_common::Trajectory makeSplineMpcReference(
    const Eigen::Vector3d& origin_pos,
    const Eigen::Vector3d& current_pos)
{
  quadrotor_common::Trajectory traj;
  traj.trajectory_type = quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  updatePathProgress(current_pos);

  const double cruise = clamp(g_cruise_speed, 0.05, g_max_speed);

  for (int i = 0; i < kHorizonPoints; ++i)
  {
    const double t = i * g_trajectory_dt;
    const double arc_s = clamp(
        g_path_progress_s + cruise * t,
        0.0,
        g_path_spline.totalLength());
    const path_spline::Sample sample =
        sampleSplineWorld(origin_pos, arc_s, cruise);

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(t);
    point.position = sample.position;
    point.velocity = sample.velocity;
    point.acceleration = sample.acceleration;
    point.orientation = Eigen::Quaterniond::Identity();
    point.bodyrates = Eigen::Vector3d::Zero();
    // rpg_mpc builds q_ref = q(heading) * orientation; heading only, no double yaw.
    point.heading = sample.velocity.norm() > 0.1
        ? std::atan2(sample.velocity.y(), sample.velocity.x())
        : 0.0;
    point.heading_rate = 0.0;
    traj.points.push_back(point);
  }
  return traj;
}

void publishReferenceVelocityWorld(const path_spline::Sample& sample)
{
  if (!g_ref_vel_pub)
  {
    return;
  }

  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "enu";
  msg.twist.linear.x = sample.velocity.x();
  msg.twist.linear.y = sample.velocity.y();
  msg.twist.linear.z = sample.velocity.z();
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;

  g_ref_vel_pub.publish(msg);
}

ControlOutputMode parseControlOutputMode(const std::string& mode)
{
  if (mode == "velocity" || mode == "vel" || mode == "vel_body")
  {
    return ControlOutputMode::kVelocity;
  }
  return ControlOutputMode::kPwm;
}

struct DroneState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  ros::Time stamp;
};

struct MpcReference
{
  Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
  double target_yaw = 0.0;
};

struct BodyCmdLimits
{
  double max_vx = 10.0;
  double max_vy = 4.0;
  double max_vz = 2.0;
  double max_yaw_rate = 1.5;
  double max_acc_step = 1.0;
};

struct MpcBodyCmdOutput
{
  airsim_ros::VelCmd cmd;
  Eigen::Vector3d v_world_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_body_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
  double yaw_error = 0.0;
  double yaw_rate_cmd = 0.0;
  bool fallback_used = false;
  bool command_valid = false;
};

class MpcBodyCmdAdapter
{
 public:
  void configure(
      double kp_pos_xy,
      double kp_pos_z,
      double kd_vel,
      double kp_yaw,
      const BodyCmdLimits& limits)
  {
    kp_pos_xy_ = kp_pos_xy;
    kp_pos_z_ = kp_pos_z;
    kd_vel_ = kd_vel;
    kp_yaw_ = kp_yaw;
    limits_ = limits;
  }

  void reset()
  {
    last_v_body_cmd_.setZero();
    last_yaw_rate_cmd_ = 0.0;
  }

  MpcBodyCmdOutput computeCommand(
      const DroneState& current_state,
      const MpcReference& reference,
      bool mpc_success,
      const Eigen::Vector3d& mpc_velocity_world,
      bool tracking_ready)
  {
    MpcBodyCmdOutput out;
    out.cmd = makeZeroCmd(current_state.stamp, false);

    if (!tracking_ready)
    {
      reset();
      return out;
    }

    const double odom_age = (ros::Time::now() - current_state.stamp).toSec();
    const bool state_ok = isFiniteVector(current_state.position) &&
        isFiniteVector(current_state.velocity) && std::isfinite(current_state.yaw) &&
        std::isfinite(odom_age) && odom_age <= 0.2;
    const bool reference_ok = isFiniteVector(reference.target_position) &&
        isFiniteVector(reference.target_velocity) &&
        std::isfinite(reference.target_yaw);
    if (!state_ok || !reference_ok)
    {
      reset();
      out.cmd = makeZeroCmd(current_state.stamp, true);
      return out;
    }
    if (mpc_success && !isFiniteVector(mpc_velocity_world))
    {
      reset();
      out.cmd = makeZeroCmd(current_state.stamp, true);
      return out;
    }

    out.position_error = reference.target_position - current_state.position;
    const Eigen::Vector3d vel_err =
        reference.target_velocity - current_state.velocity;

    Eigen::Vector3d pos_feedback(
        kp_pos_xy_ * out.position_error.x(),
        kp_pos_xy_ * out.position_error.y(),
        kp_pos_z_ * out.position_error.z());
    const Eigen::Vector3d vel_feedback = kd_vel_ * vel_err;

    if (mpc_success)
    {
      out.v_world_cmd = mpc_velocity_world + pos_feedback + vel_feedback;
    }
    else
    {
      out.fallback_used = true;
      out.v_world_cmd = reference.target_velocity + pos_feedback + vel_feedback;
    }

    out.yaw_error = normalizeAngle(reference.target_yaw - current_state.yaw);
    out.yaw_rate_cmd = kp_yaw_ * out.yaw_error;
    out.yaw_rate_cmd =
        clamp(out.yaw_rate_cmd, -limits_.max_yaw_rate, limits_.max_yaw_rate);

    Eigen::Vector3d v_body_flu =
        worldVelToBodyVel(out.v_world_cmd, current_state.yaw);

    double dynamic_max_vx = limits_.max_vx;
    const double yaw_abs = std::abs(out.yaw_error);
    if (yaw_abs > 0.7)
    {
      dynamic_max_vx = 5.0;
    }
    if (yaw_abs > 1.0)
    {
      dynamic_max_vx = 3.5;
    }
    if (out.position_error.head<2>().norm() > 8.0)
    {
      dynamic_max_vx = std::min(dynamic_max_vx, 5.0);
    }

    v_body_flu.x() = clamp(v_body_flu.x(), -dynamic_max_vx, dynamic_max_vx);
    v_body_flu.y() = clamp(v_body_flu.y(), -limits_.max_vy, limits_.max_vy);
    v_body_flu.z() = clamp(v_body_flu.z(), -limits_.max_vz, limits_.max_vz);

    Eigen::Vector3d v_body_cmd = fluBodyVelToSimCmd(v_body_flu);
    v_body_cmd.x() = rateLimit(v_body_cmd.x(), last_v_body_cmd_.x());
    v_body_cmd.y() = rateLimit(v_body_cmd.y(), last_v_body_cmd_.y());
    v_body_cmd.z() = rateLimit(v_body_cmd.z(), last_v_body_cmd_.z());
    out.yaw_rate_cmd = rateLimit(out.yaw_rate_cmd, last_yaw_rate_cmd_);

    last_v_body_cmd_ = v_body_cmd;
    last_yaw_rate_cmd_ = out.yaw_rate_cmd;

    out.v_body_cmd = v_body_cmd;
    out.cmd.header.stamp = current_state.stamp;
    out.cmd.header.frame_id = "body";
    out.cmd.vx = v_body_cmd.x();
    out.cmd.vy = v_body_cmd.y();
    out.cmd.vz = v_body_cmd.z();
    out.cmd.yawRate = g_vel_yaw_rate_in_deg
        ? out.yaw_rate_cmd * 180.0 / kPi
        : out.yaw_rate_cmd;
    out.cmd.va = static_cast<uint8_t>(clamp(g_vel_cmd_va, 0, 255));
    out.cmd.stop = 0;
    out.command_valid = true;
    return out;
  }

 private:
  airsim_ros::VelCmd makeZeroCmd(const ros::Time& stamp, bool stop) const
  {
    airsim_ros::VelCmd cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = "body";
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.vz = 0.0;
    cmd.yawRate = 0.0;
    cmd.va = 0;
    cmd.stop = stop ? 1 : 0;
    return cmd;
  }

  double rateLimit(double value, double last_value) const
  {
    return clamp(
        value,
        last_value - limits_.max_acc_step,
        last_value + limits_.max_acc_step);
  }

  double kp_pos_xy_ = 0.8;
  double kp_pos_z_ = 0.6;
  double kd_vel_ = 0.15;
  double kp_yaw_ = 1.8;
  BodyCmdLimits limits_;
  Eigen::Vector3d last_v_body_cmd_ = Eigen::Vector3d::Zero();
  double last_yaw_rate_cmd_ = 0.0;
};

MpcBodyCmdAdapter g_mpc_body_cmd_adapter;

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
    if (g_control_output_mode == ControlOutputMode::kVelocity && g_vel_cmd_pub)
    {
      airsim_ros::VelCmd zero_cmd;
      zero_cmd.header.stamp = ros::Time::now();
      zero_cmd.header.frame_id = "body";
      zero_cmd.vx = 0.0;
      zero_cmd.vy = 0.0;
      zero_cmd.vz = 0.0;
      zero_cmd.yawRate = 0.0;
      zero_cmd.va = 0;
      zero_cmd.stop = g_has_odom ? 0 : 1;
      g_vel_cmd_pub.publish(zero_cmd);
      g_mpc_body_cmd_adapter.reset();
    }
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
    g_nearest_wp_idx = 0;
    g_nearest_arc_s = 0.0;
    g_path_progress_s = 0.0;
    g_control_start_time = ros::Time::now();
    g_control_enabled = true;

    if (g_align_path_yaw_to_body && g_path_spline.numWaypoints() >= 2)
    {
      const path_spline::Sample path_start =
          g_path_spline.sampleAtArcLength(0.0, 0.0);
      double path_yaw = 0.0;
      if (path_start.velocity.head<2>().norm() > 0.05)
      {
        path_yaw = std::atan2(path_start.velocity.y(), path_start.velocity.x());
      }
      const double body_yaw = quaternionToRpy(state.orientation).z();
      g_path_yaw_offset = wrapAngle(body_yaw - path_yaw);
      ROS_INFO(
          "Path yaw aligned to body: path_yaw=%.1f deg body_yaw=%.1f deg "
          "offset=%.1f deg",
          path_yaw * 180.0 / kPi,
          body_yaw * 180.0 / kPi,
          g_path_yaw_offset * 180.0 / kPi);
    }

    ROS_INFO(
        "MPC spline control enabled. Reference origin: [%.2f %.2f %.2f], "
        "path_length=%.2f m",
        g_ref_origin_pos.x(),
        g_ref_origin_pos.y(),
        g_ref_origin_pos.z(),
        g_path_spline.totalLength());
  }

  const quadrotor_common::Trajectory reference_spline =
      makeSplineMpcReference(g_ref_origin_pos, state.position);
  quadrotor_common::Trajectory reference = reference_spline;
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
  else if (since_control_enable <
           kPostEnableZHoldSec + kPostEnableZRampSec)
  {
    const double ramp_t =
        (since_control_enable - kPostEnableZHoldSec) / kPostEnableZRampSec;
    const double alpha = clamp(ramp_t, 0.0, 1.0);
    auto ref_it = reference.points.begin();
    auto spline_it = reference_spline.points.begin();
    for (; ref_it != reference.points.end() &&
           spline_it != reference_spline.points.end();
         ++ref_it, ++spline_it)
    {
      const double z_spline = spline_it->position.z();
      ref_it->position.z() =
          (1.0 - alpha) * g_ref_origin_pos.z() + alpha * z_spline;
      ref_it->velocity.z() = spline_it->velocity.z() * alpha;
      ref_it->acceleration.z() = spline_it->acceleration.z() * alpha;
    }
  }
  const quadrotor_common::TrajectoryPoint& ref_point = reference.points.front();
  const path_spline::Sample ref_sample{
      ref_point.position,
      ref_point.velocity,
      ref_point.acceleration};
  if (g_control_output_mode != ControlOutputMode::kVelocity)
  {
    publishReferenceVelocityWorld(ref_sample);
  }
  const Eigen::Vector3d target_pos = ref_point.position;
  const Eigen::Vector3d pos_error = target_pos - state.position;
  const path_spline::Sample on_path_sample = sampleSplineWorld(
      g_ref_origin_pos,
      g_nearest_arc_s,
      g_cruise_speed);
  const Eigen::Vector3d track_error = on_path_sample.position - state.position;
  const Eigen::Vector3d vel_error = on_path_sample.velocity - state.velocity;
  double path_heading = ref_point.heading;
  if (on_path_sample.velocity.head<2>().norm() > 0.05)
  {
    path_heading = std::atan2(
        on_path_sample.velocity.y(), on_path_sample.velocity.x());
  }
  const Eigen::Vector3d rpy = quaternionToRpy(state.orientation);
  const Eigen::Vector3d target_rpy = quaternionToRpy(ref_point.orientation);

  quadrotor_common::ControlCommand cmd = g_mpc->run(state, reference, g_mpc_params);

  if (g_control_output_mode == ControlOutputMode::kPwm)
  {
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
  }

  double total_thrust = 0.0;
  Eigen::Vector3d desired_bodyrates = Eigen::Vector3d::Zero();
  Eigen::Vector3d bodyrate_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d bodyrate_error_dot = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_torque = Eigen::Vector3d::Zero();
  Eigen::Vector4d motor_force = Eigen::Vector4d::Zero();
  airsim_ros::RotorPWM pwm;
  MpcBodyCmdOutput body_cmd_out;

  if (g_control_output_mode == ControlOutputMode::kVelocity)
  {
    const int mpc_node = std::max(0, std::min(g_vel_mpc_node_index, 10));
    const Eigen::Vector3d v_mpc =
        g_mpc->getPredictedVelocity(mpc_node).template cast<double>();
    const DroneState current_state{
        state.position,
        state.velocity,
        rpy.z(),
        state.timestamp};
    const MpcReference mpc_reference{
        target_pos,
        ref_point.velocity,
        path_heading};
    const bool mpc_success = isFiniteVector(v_mpc);
    body_cmd_out = g_mpc_body_cmd_adapter.computeCommand(
        current_state,
        mpc_reference,
        mpc_success,
        v_mpc,
        g_takeoff_done && g_control_enabled);
    g_vel_cmd_pub.publish(body_cmd_out.cmd);

    geometry_msgs::TwistStamped v_world_msg;
    v_world_msg.header.stamp = ros::Time::now();
    v_world_msg.header.frame_id = "enu";
    v_world_msg.twist.linear.x = body_cmd_out.v_world_cmd.x();
    v_world_msg.twist.linear.y = body_cmd_out.v_world_cmd.y();
    v_world_msg.twist.linear.z = body_cmd_out.v_world_cmd.z();
    v_world_msg.twist.angular.z = body_cmd_out.yaw_rate_cmd;
    if (g_ref_vel_pub)
    {
      g_ref_vel_pub.publish(v_world_msg);
    }
  }
  else
  {
    pwm = mpcCommandToPwm(
        state,
        cmd,
        &total_thrust,
        &desired_bodyrates,
        &bodyrate_error,
        &bodyrate_error_dot,
        &desired_torque,
        &motor_force);
    g_pwm_pub.publish(pwm);
  }

  if (g_control_output_mode == ControlOutputMode::kVelocity)
  {
    ROS_INFO_THROTTLE(
        0.2,
        "[MPC_BODY_CMD] cur=(%.2f %.2f %.2f) tgt=(%.2f %.2f %.2f) "
        "err_xy=%.2f v_world=(%.2f %.2f %.2f) v_body=(%.2f %.2f %.2f) "
        "yaw=%.2f tgt_yaw=%.2f yaw_err=%.2f yawRate=%.2f mpc_success=%d fallback=%d "
        "wp=%zu/%zu s=%.2f/%.2f mpc_vel=(%.2f %.2f %.2f) nearest_s=%.2f",
        state.position.x(),
        state.position.y(),
        state.position.z(),
        target_pos.x(),
        target_pos.y(),
        target_pos.z(),
        body_cmd_out.position_error.head<2>().norm(),
        body_cmd_out.v_world_cmd.x(),
        body_cmd_out.v_world_cmd.y(),
        body_cmd_out.v_world_cmd.z(),
        body_cmd_out.v_body_cmd.x(),
        body_cmd_out.v_body_cmd.y(),
        body_cmd_out.v_body_cmd.z(),
        rpy.z(),
        path_heading,
        body_cmd_out.yaw_error,
        body_cmd_out.yaw_rate_cmd,
        static_cast<int>(!body_cmd_out.fallback_used),
        static_cast<int>(body_cmd_out.fallback_used),
        g_nearest_wp_idx,
        g_path_spline.numWaypoints(),
        g_path_progress_s,
        g_path_spline.totalLength(),
        g_mpc->getPredictedVelocity(std::max(0, std::min(g_vel_mpc_node_index, 10))).x(),
        g_mpc->getPredictedVelocity(std::max(0, std::min(g_vel_mpc_node_index, 10))).y(),
        g_mpc->getPredictedVelocity(std::max(0, std::min(g_vel_mpc_node_index, 10))).z(),
        g_nearest_arc_s);
    return;
  }

  ROS_INFO_THROTTLE(
      0.5,
      "MPC cmd: thrust=%.2f, rates=[%.2f %.2f %.2f], "
      "wp=%zu/%zu s=%.2f/%.2f, pos=[%.2f %.2f %.2f], target=[%.2f %.2f %.2f], "
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
      g_nearest_wp_idx,
      g_path_spline.numWaypoints(),
      g_path_progress_s,
      g_path_spline.totalLength(),
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
  setParamIfMissing("trajectory_dt", d.trajectory_dt);
  setParamIfMissing("cruise_speed", d.cruise_speed);
  setParamIfMissing("waypoint_stride", d.waypoint_stride);
  setParamIfMissing("trajectory_smooth_window", d.trajectory_smooth_window);
  setParamIfMissing("waypoint_max_segment_m", d.waypoint_max_segment_m);
  setParamIfMissing("align_trajectory_to_start", d.align_trajectory_to_start);
  setParamIfMissing("reference_lookahead_points", d.reference_lookahead_points);
  setParamIfMissing("nearest_search_back", static_cast<int>(d.nearest_search_back));
  setParamIfMissing("nearest_search_ahead", static_cast<int>(d.nearest_search_ahead));
  setParamIfMissing("max_speed", d.max_speed);
  setParamIfMissing("max_accel", d.max_accel);
  setParamIfMissing("odom_ned_to_enu", d.odom_ned_to_enu);
  setParamIfMissing("max_progress_ahead_m", d.max_progress_ahead_m);
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
  setParamIfMissing("control_output", d.control_output);
  setParamIfMissing("vel_cmd_topic", d.vel_cmd_topic);
  setParamIfMissing("vel_mpc_node_index", d.vel_mpc_node_index);
  setParamIfMissing("mpc_body_kp_pos_xy", d.mpc_body_kp_pos_xy);
  setParamIfMissing("mpc_body_kp_pos_z", d.mpc_body_kp_pos_z);
  setParamIfMissing("mpc_body_kd_vel", d.mpc_body_kd_vel);
  setParamIfMissing("mpc_body_kp_yaw", d.mpc_body_kp_yaw);
  setParamIfMissing("body_cmd_max_vx", d.body_cmd_max_vx);
  setParamIfMissing("body_cmd_max_vy", d.body_cmd_max_vy);
  setParamIfMissing("body_cmd_max_vz", d.body_cmd_max_vz);
  setParamIfMissing("body_cmd_max_yaw_rate", d.body_cmd_max_yaw_rate);
  setParamIfMissing("body_cmd_max_acc_step", d.body_cmd_max_acc_step);
  setParamIfMissing("vel_cmd_flip_body_y", d.vel_cmd_flip_body_y);
  setParamIfMissing("vel_yaw_rate_in_deg", d.vel_yaw_rate_in_deg);
  setParamIfMissing("vel_cmd_va", d.vel_cmd_va);

  std::string control_output = d.control_output;
  pnh.param("control_output", control_output, control_output);
  g_control_output_mode = parseControlOutputMode(control_output);

  pnh.param("vel_cmd_topic", g_vel_cmd_topic, g_vel_cmd_topic);
  pnh.param("vel_mpc_node_index", g_vel_mpc_node_index, g_vel_mpc_node_index);
  pnh.param("mpc_body_kp_pos_xy", g_mpc_body_kp_pos_xy, g_mpc_body_kp_pos_xy);
  pnh.param("mpc_body_kp_pos_z", g_mpc_body_kp_pos_z, g_mpc_body_kp_pos_z);
  pnh.param("mpc_body_kd_vel", g_mpc_body_kd_vel, g_mpc_body_kd_vel);
  pnh.param("mpc_body_kp_yaw", g_mpc_body_kp_yaw, g_mpc_body_kp_yaw);
  pnh.param("body_cmd_max_vx", g_body_cmd_max_vx, g_body_cmd_max_vx);
  pnh.param("body_cmd_max_vy", g_body_cmd_max_vy, g_body_cmd_max_vy);
  pnh.param("body_cmd_max_vz", g_body_cmd_max_vz, g_body_cmd_max_vz);
  pnh.param(
      "body_cmd_max_yaw_rate",
      g_body_cmd_max_yaw_rate,
      g_body_cmd_max_yaw_rate);
  pnh.param(
      "body_cmd_max_acc_step",
      g_body_cmd_max_acc_step,
      g_body_cmd_max_acc_step);
  pnh.param("along_lag_lookahead_scale", g_along_lag_lookahead_scale, g_along_lag_lookahead_scale);
  pnh.param("cross_slowdown_m", g_cross_slowdown_m, g_cross_slowdown_m);
  pnh.param(
      "cross_progress_hold_m",
      g_cross_progress_hold_m,
      g_cross_progress_hold_m);
  pnh.param("vel_cmd_flip_body_y", g_vel_cmd_flip_body_y, g_vel_cmd_flip_body_y);
  pnh.param("max_cross_track_m", g_max_cross_track_m, g_max_cross_track_m);
  pnh.param("vel_yaw_rate_in_deg", g_vel_yaw_rate_in_deg, g_vel_yaw_rate_in_deg);
  pnh.param("align_path_yaw_to_body", g_align_path_yaw_to_body, g_align_path_yaw_to_body);
  pnh.param("vel_cmd_va", g_vel_cmd_va, g_vel_cmd_va);

  std::string ref_vel_topic = "/mpc/spline_reference_vel";
  pnh.param("spline_reference_vel_topic", ref_vel_topic, ref_vel_topic);
  g_ref_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(ref_vel_topic, 1);
  ROS_INFO("Publishing spline reference velocity on %s (geometry_msgs/TwistStamped, frame=enu)",
           ref_vel_topic.c_str());

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
  pnh.param("min_thrust", g_min_thrust_acc, g_min_thrust_acc);
  pnh.param("max_thrust", g_max_thrust_acc, g_max_thrust_acc);

  g_trajectory_dt = d.trajectory_dt;
  g_cruise_speed = d.cruise_speed;
  g_waypoint_stride = d.waypoint_stride;
  g_trajectory_smooth_window = d.trajectory_smooth_window;
  g_align_trajectory_to_start = d.align_trajectory_to_start;
  g_reference_lookahead_points = d.reference_lookahead_points;
  g_nearest_search_back = d.nearest_search_back;
  g_nearest_search_ahead = d.nearest_search_ahead;
  g_max_speed = d.max_speed;
  g_max_accel = d.max_accel;

  pnh.param("trajectory_dt", g_trajectory_dt, g_trajectory_dt);
  pnh.param("cruise_speed", g_cruise_speed, g_cruise_speed);
  pnh.param("waypoint_stride", g_waypoint_stride, g_waypoint_stride);
  pnh.param(
      "trajectory_smooth_window",
      g_trajectory_smooth_window,
      g_trajectory_smooth_window);
  pnh.param(
      "waypoint_max_segment_m", g_waypoint_max_segment_m, g_waypoint_max_segment_m);
  pnh.param(
      "align_trajectory_to_start",
      g_align_trajectory_to_start,
      g_align_trajectory_to_start);
  pnh.param(
      "reference_lookahead_points",
      g_reference_lookahead_points,
      g_reference_lookahead_points);
  int nearest_back = static_cast<int>(g_nearest_search_back);
  int nearest_ahead = static_cast<int>(g_nearest_search_ahead);
  pnh.param("nearest_search_back", nearest_back, nearest_back);
  pnh.param("nearest_search_ahead", nearest_ahead, nearest_ahead);
  g_nearest_search_back = static_cast<std::size_t>(std::max(0, nearest_back));
  g_nearest_search_ahead =
      static_cast<std::size_t>(std::max(0, nearest_ahead));
  pnh.param("max_speed", g_max_speed, g_max_speed);
  pnh.param("max_accel", g_max_accel, g_max_accel);
  pnh.param("path_lookahead_m", g_path_lookahead_m, g_path_lookahead_m);
  pnh.param("max_progress_ahead_m", g_max_progress_ahead_m, g_max_progress_ahead_m);
  pnh.param("odom_ned_to_enu", g_odom_ned_to_enu, g_odom_ned_to_enu);
  g_trajectory_dt = std::max(0.02, g_trajectory_dt);
  g_cruise_speed = clamp(g_cruise_speed, 0.05, g_max_speed);
  BodyCmdLimits body_cmd_limits;
  body_cmd_limits.max_vx = g_body_cmd_max_vx;
  body_cmd_limits.max_vy = g_body_cmd_max_vy;
  body_cmd_limits.max_vz = g_body_cmd_max_vz;
  body_cmd_limits.max_yaw_rate = g_body_cmd_max_yaw_rate;
  body_cmd_limits.max_acc_step = g_body_cmd_max_acc_step;
  g_mpc_body_cmd_adapter.configure(
      g_mpc_body_kp_pos_xy,
      g_mpc_body_kp_pos_z,
      g_mpc_body_kd_vel,
      g_mpc_body_kp_yaw,
      body_cmd_limits);

  if (g_control_output_mode == ControlOutputMode::kVelocity)
  {
    g_vel_cmd_pub = nh.advertise<airsim_ros::VelCmd>(g_vel_cmd_topic, 1);
    ROS_INFO(
        "Control output: velocity -> %s (MPC world vel -> MpcBodyCmdAdapter, "
        "mpc_node=%d, kp=[%.2f %.2f %.2f %.2f], limits=[%.1f %.1f %.1f %.1f step=%.2f])",
        g_vel_cmd_topic.c_str(),
        g_vel_mpc_node_index,
        g_mpc_body_kp_pos_xy,
        g_mpc_body_kp_pos_z,
        g_mpc_body_kd_vel,
        g_mpc_body_kp_yaw,
        body_cmd_limits.max_vx,
        body_cmd_limits.max_vy,
        body_cmd_limits.max_vz,
        body_cmd_limits.max_yaw_rate,
        body_cmd_limits.max_acc_step);
  }
  else
  {
    g_pwm_pub = nh.advertise<airsim_ros::RotorPWM>(
        "/airsim_node/drone_1/rotor_pwm_cmd", 1);
    ROS_INFO("Control output: pwm -> /airsim_node/drone_1/rotor_pwm_cmd");
  }

  loadTrajectorySpline(trajectory_file);

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

  const char* output_mode_str =
      (g_control_output_mode == ControlOutputMode::kVelocity) ? "velocity" : "pwm";
  ROS_INFO(
      "controller_test MPC->%s build: control_output=%s, vel_topic=%s, mass=%.3f, arm=%.3f, "
      "I=[%.6f %.6f %.6f], Ct=%.9f, Cq=%.9f, Fmax=%.3f, "
      "rate_kp=[%.2f %.2f %.2f], rate_kd=[%.3f %.3f %.3f], "
      "thrust_acc=[%.2f %.2f], "
      "pwm_limit=[%.3f %.3f], cruise=%.2f, ned_to_enu=%d, horizon=%d, dt=%.2f, Q=[%.1f %.1f %.1f %.1f], R=[%.1f %.1f %.1f], "
      "max_bodyrate=[%.2f %.2f], err_dot_limit=%.1f, z_hold=%.1f z_ramp=%.1f, takeoff_settle=%.1f",
      output_mode_str,
      output_mode_str,
      g_vel_cmd_topic.c_str(),
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
      g_cruise_speed,
      static_cast<int>(g_odom_ned_to_enu),
      kHorizonPoints,
      g_trajectory_dt,
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
      kPostEnableZRampSec,
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
