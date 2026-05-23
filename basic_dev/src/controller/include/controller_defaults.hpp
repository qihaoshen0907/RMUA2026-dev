#pragma once

#include <string>

namespace controller_defaults
{

struct Defaults
{
  // Trajectory.
  const std::string trajectory_file = "/home/bai/RUMA_mpc1/trajectory_for_mpc.txt";

  // Vehicle and mixer.
  const double mass = 0.9;
  const double arm_length = 0.18;
  const double Ixx = 0.0046890742;
  const double Iyy = 0.0069312;
  const double Izz = 0.010421166;
  const double Ct = 0.00036771704516278653;
  const double Cq = 4.888486266072161e-06;
  const double Fmax_per_rotor = 12.538338804;

  // Rate loop and PWM (conservative baseline for stable flight).
  const double rate_kp_x = 1.8;
  const double rate_kp_y = 1.8;
  const double rate_kp_z = 0.8;
  const double rate_kd_x = 0.10;
  const double rate_kd_y = 0.10;
  const double rate_kd_z = 0.05;
  const double min_pwm = 0.10;
  const double max_pwm = 1.0;

  // MPC costs (less aggressive posture corrections, smoother input usage).
  const double Q_pos_xy = 80.0;
  const double Q_pos_z = 260.0;
  const double Q_attitude = 15.0;
  const double Q_velocity = 12.0;
  const double Q_perception = 0.0;
  const double R_thrust = 4.0;
  const double R_pitchroll = 10.0;
  const double R_yaw = 12.0;

  // MPC limits (keep above free-fall while preserving descent authority).
  const double min_thrust = 9.6;
  const double max_thrust = 13.0;
  const double max_bodyrate_xy = 0.20;
  const double max_bodyrate_z = 0.15;

  // Reference velocity clamp for file-provided vx/vy/vz.
  const double ref_vel_max = 0.5;

  // RMUA odometry often uses z-down convention; invert for MPC z-up state.
  const bool invert_odom_z = true;
};

inline const Defaults kDefaults{};

}  // namespace controller_defaults
