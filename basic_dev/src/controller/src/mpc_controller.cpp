#include "mpc_controller.hpp"

#include <algorithm>
#include <cmath>

namespace
{
template <typename T>
T clampValue(T value, T lower, T upper)
{
    return std::max(lower, std::min(upper, value));
}
}  // namespace

QuadrotorLinearMPC::QuadrotorLinearMPC(const QuadrotorPhysicalParams& params, const MPCConfig& config)
    : params_(params), config_(config)
{
    buildDiscreteModel();

    last_applied_u_.setZero();
    last_applied_u_[0] = params_.mass * params_.gravity;

    u_warm_start_.assign(config_.horizon, last_applied_u_);
}

void QuadrotorLinearMPC::setWeights(const MPCWeights& weights)
{
    weights_ = weights;
}

void QuadrotorLinearMPC::setConstraints(const MPCConstraints& constraints)
{
    constraints_ = constraints;
}

void QuadrotorLinearMPC::buildDiscreteModel()
{
    const double dt = config_.dt;

    A_.setIdentity();
    B_.setZero();
    affine_g_.setZero();

    // Position propagation.
    A_(0, 3) = dt;
    A_(1, 4) = dt;
    A_(2, 5) = dt;

    // Small-angle translational coupling (FLU frame).
    A_(3, 7) = params_.gravity * dt;   // ax ~= g * theta
    A_(4, 6) = -params_.gravity * dt;  // ay ~= -g * phi

    // Vertical acceleration from collective thrust.
    B_(5, 0) = dt / params_.mass;  // az ~= T / m - g
    affine_g_(5) = -params_.gravity * dt;

    // Euler angle propagation from body rates.
    A_(6, 9) = dt;
    A_(7, 10) = dt;
    A_(8, 11) = dt;

    // Body-rate propagation from torques.
    B_(9, 1) = dt / params_.Ixx;
    B_(10, 2) = dt / params_.Iyy;
    B_(11, 3) = dt / params_.Izz;
}

QuadrotorLinearMPC::State QuadrotorLinearMPC::clampState(const State& state) const
{
    State clamped = state;

    const double acc_based_angle = std::atan2(std::max(0.0, constraints_.max_acc_xy), params_.gravity);
    const double max_roll_pitch = std::min(constraints_.max_roll_pitch, acc_based_angle);
    clamped[6] = clampValue(clamped[6], -max_roll_pitch, max_roll_pitch);
    clamped[7] = clampValue(clamped[7], -max_roll_pitch, max_roll_pitch);
    clamped[11] = clampValue(clamped[11], -constraints_.max_yaw_rate, constraints_.max_yaw_rate);

    const double vel_xy = std::sqrt(clamped[3] * clamped[3] + clamped[4] * clamped[4]);
    if (vel_xy > constraints_.max_vel_xy && vel_xy > 1e-6)
    {
        const double scale = constraints_.max_vel_xy / vel_xy;
        clamped[3] *= scale;
        clamped[4] *= scale;
    }
    clamped[5] = clampValue(clamped[5], -constraints_.max_vel_z, constraints_.max_vel_z);

    return clamped;
}

void QuadrotorLinearMPC::clampControl(Control& u, const Control& prev_u) const
{
    const double acc_min_thrust = params_.mass * (params_.gravity - constraints_.max_acc_z);
    const double acc_max_thrust = params_.mass * (params_.gravity + constraints_.max_acc_z);
    double thrust_lower = std::max(constraints_.min_total_thrust, acc_min_thrust);
    double thrust_upper = std::min(constraints_.max_total_thrust, acc_max_thrust);
    if (thrust_lower > thrust_upper)
    {
        std::swap(thrust_lower, thrust_upper);
    }

    u[0] = clampValue(u[0], thrust_lower, thrust_upper);
    u[1] = clampValue(u[1], -constraints_.max_torque_x, constraints_.max_torque_x);
    u[2] = clampValue(u[2], -constraints_.max_torque_y, constraints_.max_torque_y);
    u[3] = clampValue(u[3], -constraints_.max_torque_z, constraints_.max_torque_z);

    // Slew limits for smooth outputs.
    const double dt = std::max(1e-4, config_.dt);
    const double max_d_thrust = constraints_.max_thrust_rate * dt;
    const double max_d_torque = constraints_.max_torque_rate * dt;

    u[0] = clampValue(u[0], prev_u[0] - max_d_thrust, prev_u[0] + max_d_thrust);
    u[1] = clampValue(u[1], prev_u[1] - max_d_torque, prev_u[1] + max_d_torque);
    u[2] = clampValue(u[2], prev_u[2] - max_d_torque, prev_u[2] + max_d_torque);
    u[3] = clampValue(u[3], prev_u[3] - max_d_torque, prev_u[3] + max_d_torque);
}

QuadrotorLinearMPC::State QuadrotorLinearMPC::propagate(const State& x, const Control& u) const
{
    return clampState(A_ * x + B_ * u + affine_g_);
}

QuadrotorLinearMPC::Control QuadrotorLinearMPC::controlReferenceFromState(const State& x_ref) const
{
    (void)x_ref;
    Control u_ref = Control::Zero();
    u_ref[0] = params_.mass * params_.gravity;  // hover thrust feedforward
    return u_ref;
}

double QuadrotorLinearMPC::computeCost(const std::vector<State>& x_traj,
                                       const std::vector<Control>& u_traj,
                                       const std::vector<State>& x_ref_traj) const
{
    const Eigen::Matrix<double, 12, 12> Q = weights_.Q_diag.asDiagonal();
    const Eigen::Matrix<double, 12, 12> Qf = weights_.Qf_diag.asDiagonal();
    const Eigen::Matrix<double, 4, 4> R = weights_.R_diag.asDiagonal();
    const Eigen::Matrix<double, 4, 4> Rd = weights_.Rd_diag.asDiagonal();

    double total_cost = 0.0;
    for (int k = 0; k < config_.horizon; ++k)
    {
        const State x_err = x_traj[k] - x_ref_traj[k];
        const Control u_ref = controlReferenceFromState(x_ref_traj[k]);
        const Control u_err = u_traj[k] - u_ref;
        total_cost += (x_err.transpose() * Q * x_err)(0, 0);
        total_cost += (u_err.transpose() * R * u_err)(0, 0);

        const Control prev_u = (k == 0) ? last_applied_u_ : u_traj[k - 1];
        const Control du = u_traj[k] - prev_u;
        total_cost += (du.transpose() * Rd * du)(0, 0);
    }

    const State x_terminal_err = x_traj.back() - x_ref_traj.back();
    total_cost += (x_terminal_err.transpose() * Qf * x_terminal_err)(0, 0);
    return total_cost;
}

Eigen::Matrix<double, 4, 1> QuadrotorLinearMPC::controlToRotorPWM(const Control& u) const
{
    // Keep the same mixer structure used by the original PD controller.
    Eigen::Matrix4d M;
    M << 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0;

    constexpr double kArmProjection = 0.7071067811865476;  // sqrt(2)/2

    Eigen::Vector4d u_scaled;
    u_scaled[0] = u[0] / params_.Ct;
    u_scaled[1] = u[1] / (params_.arm_length * params_.Ct * kArmProjection);
    u_scaled[2] = u[2] / (params_.arm_length * params_.Ct * kArmProjection);
    u_scaled[3] = u[3] / params_.Cq;

    Eigen::Vector4d rotor_thrust = params_.Ct * M.inverse() * u_scaled;
    rotor_thrust = rotor_thrust.cwiseMax(0.0);

    Eigen::Matrix<double, 4, 1> pwm = rotor_thrust / params_.Fmax;
    double max_pwm = pwm.maxCoeff();
    if (max_pwm > constraints_.max_pwm && max_pwm > 1e-6)
    {
        pwm /= max_pwm;
    }

    for (int i = 0; i < 4; ++i)
    {
        pwm[i] = clampValue(pwm[i], constraints_.min_pwm, constraints_.max_pwm);
    }
    return pwm;
}

MPCResult QuadrotorLinearMPC::solve(const State& x0, const std::vector<State>& x_ref_traj)
{
    const int N = config_.horizon;
    MPCResult result;
    result.u = last_applied_u_;
    result.rotor_pwm.setConstant(constraints_.min_pwm);

    if (N <= 0 || static_cast<int>(x_ref_traj.size()) < N + 1)
    {
        return result;
    }

    if (static_cast<int>(u_warm_start_.size()) != N)
    {
        u_warm_start_.assign(N, last_applied_u_);
    }

    std::vector<State> x_traj(N + 1, State::Zero());
    std::vector<State> lambda(N + 1, State::Zero());
    std::vector<Control> grad_u(N, Control::Zero());

    const Eigen::Matrix<double, 12, 12> Q = weights_.Q_diag.asDiagonal();
    const Eigen::Matrix<double, 12, 12> Qf = weights_.Qf_diag.asDiagonal();
    const Eigen::Matrix<double, 4, 4> R = weights_.R_diag.asDiagonal();
    const Eigen::Matrix<double, 4, 4> Rd = weights_.Rd_diag.asDiagonal();

    bool converged = false;
    for (int iter = 0; iter < config_.max_iterations; ++iter)
    {
        x_traj[0] = x0;
        for (int k = 0; k < N; ++k)
        {
            x_traj[k + 1] = propagate(x_traj[k], u_warm_start_[k]);
        }

        lambda[N] = 2.0 * Qf * (x_traj[N] - x_ref_traj[N]);
        for (int k = N - 1; k >= 0; --k)
        {
            const State x_err = x_traj[k] - x_ref_traj[k];
            const Control u_ref = controlReferenceFromState(x_ref_traj[k]);
            const Control u_err = u_warm_start_[k] - u_ref;

            Control smooth_grad = Control::Zero();
            const Control prev_u = (k == 0) ? last_applied_u_ : u_warm_start_[k - 1];
            smooth_grad += 2.0 * Rd * (u_warm_start_[k] - prev_u);
            if (k < N - 1)
            {
                smooth_grad += 2.0 * Rd * (u_warm_start_[k] - u_warm_start_[k + 1]);
            }

            grad_u[k] = 2.0 * R * u_err + B_.transpose() * lambda[k + 1] + smooth_grad;
            lambda[k] = 2.0 * Q * x_err + A_.transpose() * lambda[k + 1];
        }

        double max_step = 0.0;
        for (int k = 0; k < N; ++k)
        {
            const Control prev_control = (k == 0) ? last_applied_u_ : u_warm_start_[k - 1];
            const Control before_update = u_warm_start_[k];
            u_warm_start_[k] -= config_.gradient_step * grad_u[k];
            clampControl(u_warm_start_[k], prev_control);
            max_step = std::max(max_step, (u_warm_start_[k] - before_update).cwiseAbs().maxCoeff());
        }

        if (max_step < config_.control_tol)
        {
            converged = true;
            break;
        }
    }

    x_traj[0] = x0;
    for (int k = 0; k < N; ++k)
    {
        x_traj[k + 1] = propagate(x_traj[k], u_warm_start_[k]);
    }

    result.u = u_warm_start_.front();
    clampControl(result.u, last_applied_u_);
    result.rotor_pwm = controlToRotorPWM(result.u);
    result.final_cost = computeCost(x_traj, u_warm_start_, x_ref_traj);
    result.converged = converged;

    last_applied_u_ = result.u;

    // Shift warm start for receding horizon.
    for (int k = 0; k < N - 1; ++k)
    {
        u_warm_start_[k] = u_warm_start_[k + 1];
    }
    if (N >= 2)
    {
        u_warm_start_[N - 1] = u_warm_start_[N - 2];
    }
    else
    {
        u_warm_start_[N - 1] = last_applied_u_;
    }

    return result;
}
