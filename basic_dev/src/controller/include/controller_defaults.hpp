#pragma once

#include <string>

namespace controller_defaults
{

struct Defaults
{
  // Trajectory.
  const std::string trajectory_file = "../../../trajectory_for_mpc.txt";

  // Vehicle and mixer.
  const double mass = 0.9;
  const double arm_length = 0.18;
  const double Ixx = 0.0046890742;
  const double Iyy = 0.0069312;
  const double Izz = 0.010421166;
  const double Ct = 0.00036771704516278653;
  const double Cq = 4.888486266072161e-06;
  const double Fmax_per_rotor = 12.538338804;

  // Rate loop and PWM.
  const double rate_kp_x = 2.4;
  const double rate_kp_y = 2.4;
  const double rate_kp_z = 1.0;
  const double rate_kd_x = 0.14;
  const double rate_kd_y = 0.14;
  const double rate_kd_z = 0.06;
  const double min_pwm = 0.10;
  const double max_pwm = 1.0;

  // MPC costs.
  const double Q_pos_xy = 80.0;
  const double Q_pos_z = 260.0;
  const double Q_attitude = 35.0;
  const double Q_velocity = 12.0;
  const double Q_perception = 0.0;
  const double R_thrust = 4.0;
  const double R_pitchroll = 8.0;
  const double R_yaw = 10.0;

  const double min_thrust = 10.2;
  const double max_thrust = 13.0;
  const double max_bodyrate_xy = 0.30;
  const double max_bodyrate_z = 0.20;

  // Spline reference.
  const double trajectory_dt = 0.1;
  const double cruise_speed = 4.0;
  const int waypoint_stride = 1;
  const int trajectory_smooth_window = 3;
  const double waypoint_max_segment_m = 5.0;
  const bool align_trajectory_to_start = true;
  const int reference_lookahead_points = 1;
  const std::size_t nearest_search_back = 60;
  const std::size_t nearest_search_ahead = 200;
  const double max_speed = 4.0;
  const double max_accel = 2.5;
  const double path_lookahead_m = 0.5;
  const double max_progress_ahead_m = 3.0;
  const double cross_slowdown_m = 0.8;
  const double cross_progress_hold_m = 1.2;
  const double along_lag_lookahead_scale = 0.5;

  const bool odom_ned_to_enu = true;

  const std::string control_output = "velocity";
  const std::string vel_cmd_topic = "/airsim_node/drone_1/vel_body_cmd";
  const int vel_mpc_node_index = 1;
  const double mpc_body_kp_pos_xy = 0.8;
  const double mpc_body_kp_pos_z = 0.6;
  const double mpc_body_kd_vel = 0.15;
  const double mpc_body_kp_yaw = 1.8;
  const double body_cmd_max_vx = 10.0;
  const double body_cmd_max_vy = 4.0;
  const double body_cmd_max_vz = 2.0;
  const double body_cmd_max_yaw_rate = 1.5;
  const double body_cmd_max_acc_step = 1.0;
  const bool vel_cmd_flip_body_y = true;
  const double max_cross_track_m = 3.0;
  const bool vel_yaw_rate_in_deg = false;
  const bool align_path_yaw_to_body = true;
  const int vel_cmd_va = 3;
};

inline const Defaults kDefaults{};

}  // namespace controller_defaults
