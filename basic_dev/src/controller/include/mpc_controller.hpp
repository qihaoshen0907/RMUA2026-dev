#pragma once

#include <Eigen/Dense>
#include <vector>

struct QuadrotorPhysicalParams
{
    double mass = 0.9;
    double gravity = 9.8;
    double arm_length = 0.18;
    double Ixx = 0.0046890742;
    double Iyy = 0.0069312;
    double Izz = 0.010421166;
    double Ct = 0.00036771704516278653;
    double Cq = 4.888486266072161e-06;
    double Fmax = 4.179446268 * 3.0;
};

struct MPCWeights
{
    Eigen::Matrix<double, 12, 1> Q_diag = [] {
        Eigen::Matrix<double, 12, 1> q;
        q << 12.0, 12.0, 18.0, 2.0, 2.0, 4.0, 6.0, 6.0, 4.0, 0.8, 0.8, 1.5;
        return q;
    }();

    Eigen::Matrix<double, 12, 1> Qf_diag = [] {
        Eigen::Matrix<double, 12, 1> qf;
        qf << 20.0, 20.0, 30.0, 3.0, 3.0, 6.0, 10.0, 10.0, 8.0, 1.2, 1.2, 2.2;
        return qf;
    }();

    Eigen::Matrix<double, 4, 1> R_diag = [] {
        Eigen::Matrix<double, 4, 1> r;
        r << 0.05, 4.0, 4.0, 2.5;
        return r;
    }();

    Eigen::Matrix<double, 4, 1> Rd_diag = [] {
        Eigen::Matrix<double, 4, 1> rd;
        rd << 0.25, 8.0, 8.0, 5.0;
        return rd;
    }();
};

struct MPCConstraints
{
    double min_total_thrust = 0.0;
    double max_total_thrust = 4.0 * (4.179446268 * 3.0);

    double max_torque_x = 2.0;
    double max_torque_y = 2.0;
    double max_torque_z = 0.8;

    double max_roll_pitch = 0.35;
    double max_yaw_rate = 1.2;
    double max_vel_xy = 5.0;
    double max_vel_z = 2.0;
    double max_acc_xy = 4.0;
    double max_acc_z = 5.0;

    double max_thrust_rate = 60.0;
    double max_torque_rate = 12.0;

    double min_pwm = 0.10;
    double max_pwm = 1.00;
};

struct MPCConfig
{
    double dt = 0.02;
    int horizon = 20;
    int max_iterations = 10;
    double gradient_step = 0.03;
    double control_tol = 1e-3;
};

struct MPCResult
{
    Eigen::Matrix<double, 4, 1> u;
    Eigen::Matrix<double, 4, 1> rotor_pwm;
    double final_cost = 0.0;
    bool converged = false;
};

class QuadrotorLinearMPC
{
public:
    using State = Eigen::Matrix<double, 12, 1>;
    using Control = Eigen::Matrix<double, 4, 1>;

    QuadrotorLinearMPC(const QuadrotorPhysicalParams& params, const MPCConfig& config);

    void setWeights(const MPCWeights& weights);
    void setConstraints(const MPCConstraints& constraints);
    const MPCConfig& config() const { return config_; }

    MPCResult solve(const State& x0, const std::vector<State>& x_ref_traj);

private:
    void buildDiscreteModel();
    State clampState(const State& state) const;
    void clampControl(Control& u, const Control& prev_u) const;
    State propagate(const State& x, const Control& u) const;
    double computeCost(const std::vector<State>& x_traj,
                       const std::vector<Control>& u_traj,
                       const std::vector<State>& x_ref_traj) const;
    Control controlReferenceFromState(const State& x_ref) const;
    Eigen::Matrix<double, 4, 1> controlToRotorPWM(const Control& u) const;

private:
    QuadrotorPhysicalParams params_;
    MPCConfig config_;
    MPCWeights weights_;
    MPCConstraints constraints_;

    Eigen::Matrix<double, 12, 12> A_ = Eigen::Matrix<double, 12, 12>::Identity();
    Eigen::Matrix<double, 12, 4> B_ = Eigen::Matrix<double, 12, 4>::Zero();
    State affine_g_ = State::Zero();

    std::vector<Control> u_warm_start_;
    Control last_applied_u_ = Control::Zero();
};
