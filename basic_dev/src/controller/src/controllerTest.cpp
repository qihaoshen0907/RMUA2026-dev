#include "controllerTest.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

std::vector<Eigen::Vector3d> spline_path;
int current_wp_idx = 0;
bool spline_loaded = false;
bool g_path_complete = false;

namespace
{
constexpr double kTakeoffHoldSec = 3.0;
constexpr double kWaypointReachDist = 0.8;  // compatibility
constexpr double kWaypointReachXY = 1.0;
constexpr double kWaypointReachZ = 0.35;
constexpr double kWaypointSkipBehindX = 0.25;
constexpr double kWaypointSkipXY = 1.2;
constexpr double kWaypointSkipZ = 0.5;

constexpr double kCruiseSpeed = 2.5;
constexpr double kDensifySegLen = 0.5;
constexpr double kYawSlewRate = 0.8;  // rad/s, suppresses aggressive yaw oscillation

double s_waypoint_reach_dist = kWaypointReachDist;
double s_waypoint_reach_xy = kWaypointReachXY;
double s_waypoint_reach_z = kWaypointReachZ;
double s_waypoint_skip_behind_x = kWaypointSkipBehindX;
double s_waypoint_skip_xy = kWaypointSkipXY;
double s_waypoint_skip_z = kWaypointSkipZ;
double s_cruise_speed = kCruiseSpeed;

double s_last_ref_yaw = 0.0;
bool s_ref_yaw_initialized = false;
ros::Time s_prev_stamp;

template <typename T>
T clampValue(T value, T lower, T upper)
{
    return std::max(lower, std::min(upper, value));
}

double wrapToPi(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool isWaypointReached(const Eigen::Vector3d& cur_pos,
                       const Eigen::Vector3d& target,
                       double xy_tol,
                       double z_tol)
{
    const Eigen::Vector2d dxy = (target - cur_pos).head<2>();
    const double xy_err = dxy.norm();
    const double z_err = std::abs(target.z() - cur_pos.z());
    return (xy_err <= xy_tol && z_err <= z_tol);
}

bool shouldSkipWaypoint(const Eigen::Vector3d& cur_pos,
                        const Eigen::Vector3d& target,
                        double behind_x_tol,
                        double xy_tol,
                        double z_tol)
{
    const double dx = target.x() - cur_pos.x();
    const double dy = target.y() - cur_pos.y();
    const double dz = target.z() - cur_pos.z();
    const double xy_err = std::sqrt(dx * dx + dy * dy);
    return (dx < -behind_x_tol && xy_err < xy_tol && std::abs(dz) < z_tol);
}

std::vector<Eigen::Vector3d> densifyPath(const std::vector<Eigen::Vector3d>& path, double max_seg_len)
{
    std::vector<Eigen::Vector3d> refined_path;
    if (path.empty())
    {
        return refined_path;
    }
    if (path.size() == 1)
    {
        refined_path.push_back(path.front());
        return refined_path;
    }

    for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
    {
        const Eigen::Vector3d p0 = path[i];
        const Eigen::Vector3d p1 = path[i + 1];
        refined_path.push_back(p0);

        const Eigen::Vector3d diff = p1 - p0;
        const double dist = diff.norm();
        if (dist > max_seg_len)
        {
            const int num_segments = static_cast<int>(std::ceil(dist / max_seg_len));
            for (int k = 1; k < num_segments; ++k)
            {
                const double alpha = static_cast<double>(k) / static_cast<double>(num_segments);
                refined_path.push_back(p0 + alpha * diff);
            }
        }
    }
    refined_path.push_back(path.back());
    return refined_path;
}

std::vector<QuadrotorLinearMPC::State> buildReferenceTrajectory(const QuadrotorLinearMPC::State& x_real,
                                                                const Eigen::Vector3d& target,
                                                                double desired_yaw)
{
    const int horizon = g_mpc_controller->config().horizon;
    const double dt = g_mpc_controller->config().dt;

    std::vector<QuadrotorLinearMPC::State> refs(horizon + 1, QuadrotorLinearMPC::State::Zero());
    const Eigen::Vector3d cur_pos(x_real[0], x_real[1], x_real[2]);
    const Eigen::Vector3d err = target - cur_pos;
    const double dist = err.norm();

    Eigen::Vector3d dir = Eigen::Vector3d::Zero();
    if (dist > 1e-6)
    {
        dir = err / dist;
    }

    for (int k = 0; k <= horizon; ++k)
    {
        QuadrotorLinearMPC::State xr = QuadrotorLinearMPC::State::Zero();
        const double horizon_dist = std::min(dist, s_cruise_speed * dt * static_cast<double>(k));
        const Eigen::Vector3d ref_pos = cur_pos + dir * horizon_dist;

        xr[0] = ref_pos.x();
        xr[1] = ref_pos.y();
        xr[2] = ref_pos.z();

        const bool hold_final = (horizon_dist >= dist - 1e-3);
        const Eigen::Vector3d ref_vel = hold_final ? Eigen::Vector3d::Zero() : (dir * s_cruise_speed);
        xr[3] = ref_vel.x();
        xr[4] = ref_vel.y();
        xr[5] = ref_vel.z();

        xr[6] = 0.0;
        xr[7] = 0.0;
        xr[8] = desired_yaw;
        xr[9] = 0.0;
        xr[10] = 0.0;
        xr[11] = 0.0;
        refs[k] = xr;
    }
    return refs;
}

void publishSafePWM(const ros::Time& stamp)
{
    airsim_ros::RotorPWM pwm;
    pwm.header.stamp = stamp;
    pwm.rotorPWM0 = 0.10;
    pwm.rotorPWM1 = 0.10;
    pwm.rotorPWM2 = 0.10;
    pwm.rotorPWM3 = 0.10;
    g_pwm_publisher.publish(pwm);
}
}  // namespace

void loadSpline(const std::string& file_path)
{
    spline_path.clear();
    current_wp_idx = 0;
    g_path_complete = false;
    spline_loaded = false;

    if (!get_init_pose)
    {
        ROS_WARN("Initial pose not ready yet, cannot convert spline from global to local.");
        return;
    }

    std::ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open spline file: %s", file_path.c_str());
        return;
    }

    Eigen::Matrix4d TWfluWned;
    TWfluWned << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
    const Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    while (file >> x >> y >> z)
    {
        const Eigen::Vector4d p_world_ned(x, y, z, 1.0);
        const Eigen::Vector4d p_world_flu = TWfluWned * p_world_ned;
        const Eigen::Vector4d p_local_flu = TWflu0.inverse() * p_world_flu;
        spline_path.emplace_back(p_local_flu.x(), p_local_flu.y(), p_local_flu.z());
    }
    file.close();

    if (spline_path.empty())
    {
        ROS_ERROR("Spline file is empty.");
        return;
    }
    spline_path = densifyPath(spline_path, kDensifySegLen);
    spline_loaded = true;
    ROS_INFO("Spline loaded and transformed to local FLU. Total points: %zu", spline_path.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_test");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    get_init_pose = false;
    get_end_goal = false;
    g_takeoff_sent = false;
    g_path_complete = false;
    spline_loaded = false;
    current_wp_idx = 0;
    s_ref_yaw_initialized = false;

    const std::string default_spline = "src/controller/src/baseline.txt";
    pn.param<std::string>("spline_path", g_spline_file_path, default_spline);
    pn.param("waypoint_reach_dist", s_waypoint_reach_dist, kWaypointReachDist);
    pn.param("waypoint_reach_xy", s_waypoint_reach_xy, kWaypointReachXY);
    pn.param("waypoint_reach_z", s_waypoint_reach_z, kWaypointReachZ);
    pn.param("waypoint_skip_behind_x", s_waypoint_skip_behind_x, kWaypointSkipBehindX);
    pn.param("waypoint_skip_xy", s_waypoint_skip_xy, kWaypointSkipXY);
    pn.param("waypoint_skip_z", s_waypoint_skip_z, kWaypointSkipZ);
    pn.param("cruise_speed", s_cruise_speed, kCruiseSpeed);

    // Quadrotor physical parameters used by MPC dynamics and mixer.
    QuadrotorPhysicalParams params;
    pn.param("mass", params.mass, params.mass);
    pn.param("gravity", params.gravity, params.gravity);
    pn.param("arm_length", params.arm_length, params.arm_length);
    pn.param("Ixx", params.Ixx, params.Ixx);
    pn.param("Iyy", params.Iyy, params.Iyy);
    pn.param("Izz", params.Izz, params.Izz);
    pn.param("Ct", params.Ct, params.Ct);
    pn.param("Cq", params.Cq, params.Cq);
    pn.param("Fmax", params.Fmax, params.Fmax);

    // Practical real-time MPC configuration.
    MPCConfig mpc_config;
    pn.param("mpc_dt", mpc_config.dt, mpc_config.dt);
    pn.param("mpc_horizon", mpc_config.horizon, mpc_config.horizon);
    pn.param("mpc_max_iterations", mpc_config.max_iterations, mpc_config.max_iterations);
    pn.param("mpc_gradient_step", mpc_config.gradient_step, mpc_config.gradient_step);
    pn.param("mpc_control_tol", mpc_config.control_tol, mpc_config.control_tol);

    // MPC cost weights: [x y z vx vy vz roll pitch yaw wx wy wz].
    MPCWeights mpc_weights;
    pn.param("q_x", mpc_weights.Q_diag[0], mpc_weights.Q_diag[0]);
    pn.param("q_y", mpc_weights.Q_diag[1], mpc_weights.Q_diag[1]);
    pn.param("q_z", mpc_weights.Q_diag[2], mpc_weights.Q_diag[2]);
    pn.param("q_vx", mpc_weights.Q_diag[3], mpc_weights.Q_diag[3]);
    pn.param("q_vy", mpc_weights.Q_diag[4], mpc_weights.Q_diag[4]);
    pn.param("q_vz", mpc_weights.Q_diag[5], mpc_weights.Q_diag[5]);
    pn.param("q_roll", mpc_weights.Q_diag[6], mpc_weights.Q_diag[6]);
    pn.param("q_pitch", mpc_weights.Q_diag[7], mpc_weights.Q_diag[7]);
    pn.param("q_yaw", mpc_weights.Q_diag[8], mpc_weights.Q_diag[8]);
    pn.param("q_wx", mpc_weights.Q_diag[9], mpc_weights.Q_diag[9]);
    pn.param("q_wy", mpc_weights.Q_diag[10], mpc_weights.Q_diag[10]);
    pn.param("q_wz", mpc_weights.Q_diag[11], mpc_weights.Q_diag[11]);
    pn.param("r_thrust", mpc_weights.R_diag[0], mpc_weights.R_diag[0]);
    pn.param("r_tx", mpc_weights.R_diag[1], mpc_weights.R_diag[1]);
    pn.param("r_ty", mpc_weights.R_diag[2], mpc_weights.R_diag[2]);
    pn.param("r_tz", mpc_weights.R_diag[3], mpc_weights.R_diag[3]);

    // Constraints used during MPC optimization and command limiting.
    MPCConstraints mpc_constraints;
    pn.param("min_total_thrust", mpc_constraints.min_total_thrust, mpc_constraints.min_total_thrust);
    pn.param("max_total_thrust", mpc_constraints.max_total_thrust, mpc_constraints.max_total_thrust);
    pn.param("max_torque_x", mpc_constraints.max_torque_x, mpc_constraints.max_torque_x);
    pn.param("max_torque_y", mpc_constraints.max_torque_y, mpc_constraints.max_torque_y);
    pn.param("max_torque_z", mpc_constraints.max_torque_z, mpc_constraints.max_torque_z);
    pn.param("max_roll_pitch", mpc_constraints.max_roll_pitch, mpc_constraints.max_roll_pitch);
    pn.param("max_yaw_rate", mpc_constraints.max_yaw_rate, mpc_constraints.max_yaw_rate);
    pn.param("max_vel_xy", mpc_constraints.max_vel_xy, mpc_constraints.max_vel_xy);
    pn.param("max_vel_z", mpc_constraints.max_vel_z, mpc_constraints.max_vel_z);
    pn.param("max_acc_xy", mpc_constraints.max_acc_xy, mpc_constraints.max_acc_xy);
    pn.param("max_acc_z", mpc_constraints.max_acc_z, mpc_constraints.max_acc_z);
    pn.param("max_thrust_rate", mpc_constraints.max_thrust_rate, mpc_constraints.max_thrust_rate);
    pn.param("max_torque_rate", mpc_constraints.max_torque_rate, mpc_constraints.max_torque_rate);
    pn.param("min_pwm", mpc_constraints.min_pwm, mpc_constraints.min_pwm);
    pn.param("max_pwm", mpc_constraints.max_pwm, mpc_constraints.max_pwm);

    g_mpc_controller = std::make_unique<QuadrotorLinearMPC>(params, mpc_config);
    g_mpc_controller->setWeights(mpc_weights);
    g_mpc_controller->setConstraints(mpc_constraints);

    g_takeoff_client = n.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    g_pwm_publisher = n.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);

    ros::Subscriber odom_suber = n.subscribe<nav_msgs::Odometry>("/eskf_odom", 1, odom_cb);
    ros::Subscriber init_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, init_pose_cb);
    ros::Subscriber end_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_goal", 1, end_position_cb);

    ROS_INFO("controller_test started with MPC backend.");
    ROS_INFO("Subscriptions: /eskf_odom, /airsim_node/initial_pose, /airsim_node/end_goal");
    ROS_INFO("Publishing: /airsim_node/drone_1/rotor_pwm_cmd");
    ROS_INFO("MPC cfg: dt=%.4f horizon=%d iters=%d step=%.4f", mpc_config.dt, mpc_config.horizon, mpc_config.max_iterations, mpc_config.gradient_step);

    ros::Rate loop_rate(200.0);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    const Eigen::Matrix3d rotationM = q.normalized().toRotationMatrix();
    const Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    Tw0 = Eigen::Matrix4d::Identity();
    Tw0.block<3, 3>(0, 0) = rotationM;
    Tw0.block<3, 1>(0, 3) = pos;
    Twb_last = Tw0;
    get_init_pose = true;

    ROS_INFO_ONCE("Initial pose received.");
}

void end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Pwend = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    get_end_goal = true;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!get_init_pose || !g_mpc_controller)
    {
        return;
    }

    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);

    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twb.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    Twb(0, 3) = msg->pose.pose.position.x;
    Twb(1, 3) = msg->pose.pose.position.y;
    Twb(2, 3) = msg->pose.pose.position.z;

    // Keep existing NED->FLU and Tw0 local-frame conversion.
    Eigen::Matrix4d TWfluWned;
    TWfluWned << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;

    const Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();
    const Eigen::Matrix4d TWflub = TWfluWned * Twb * TWfluWned.inverse();
    const Eigen::Matrix4d T0flub = TWflu0.inverse() * TWflub;

    const Eigen::Vector3d VWned(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const Eigen::Vector3d VBned = Twb.block<3, 3>(0, 0).transpose() * VWned;
    const Eigen::Vector3d VBflu = TWfluWned.block<3, 3>(0, 0) * VBned;

    const Eigen::Vector3d Wned(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    const Eigen::Vector3d Wflu = TWfluWned.block<3, 3>(0, 0) * Wned;

    const double phi = std::asin(clampValue(T0flub(2, 1), -1.0, 1.0));
    const double cos_phi = std::max(1e-4, std::cos(phi));
    const double theta = std::atan2(-T0flub(2, 0) / cos_phi, T0flub(2, 2) / cos_phi);
    const double psi = std::atan2(-T0flub(0, 1) / cos_phi, T0flub(1, 1) / cos_phi);

    // 12D state vector:
    // [x y z vx vy vz roll pitch yaw wx wy wz].
    QuadrotorLinearMPC::State X_real = QuadrotorLinearMPC::State::Zero();
    X_real << T0flub(0, 3), T0flub(1, 3), T0flub(2, 3), VBflu.x(), VBflu.y(), VBflu.z(), phi, theta, psi, Wflu.x(), Wflu.y(), Wflu.z();

    if (!g_takeoff_sent)
    {
        airsim_ros::Takeoff tf_cmd;
        tf_cmd.request.waitOnLastTask = 1;
        if (g_takeoff_client.call(tf_cmd))
        {
            g_takeoff_sent = true;
            g_takeoff_time = ros::Time::now();
            ROS_INFO("Takeoff called. Hold %.1f s before loading spline.", kTakeoffHoldSec);
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "Takeoff service call failed, retrying...");
        }
        publishSafePWM(msg->header.stamp);
        return;
    }

    if (g_takeoff_sent && !spline_loaded && (ros::Time::now() - g_takeoff_time).toSec() >= kTakeoffHoldSec)
    {
        loadSpline(g_spline_file_path);
    }

    const Eigen::Vector3d cur_pos(X_real[0], X_real[1], X_real[2]);
    Eigen::Vector3d target = cur_pos;

    if (spline_loaded && !spline_path.empty())
    {
        while (current_wp_idx < static_cast<int>(spline_path.size()) - 1)
        {
            const Eigen::Vector3d& wp = spline_path[current_wp_idx];
            const bool reached = isWaypointReached(cur_pos, wp, s_waypoint_reach_xy, s_waypoint_reach_z);
            const bool skipped = shouldSkipWaypoint(cur_pos, wp, s_waypoint_skip_behind_x, s_waypoint_skip_xy, s_waypoint_skip_z);
            if (reached || skipped)
            {
                ++current_wp_idx;
            }
            else
            {
                break;
            }
        }

        if (current_wp_idx >= static_cast<int>(spline_path.size()))
        {
            g_path_complete = true;
            target = spline_path.back();
        }
        else
        {
            target = spline_path[current_wp_idx];
        }
    }

    const Eigen::Vector3d pos_err = target - cur_pos;
    const double xy_err = pos_err.head<2>().norm();
    double desired_yaw = psi;
    if (xy_err > 0.20)
    {
        desired_yaw = std::atan2(pos_err.y(), pos_err.x());
    }
    if (!s_ref_yaw_initialized)
    {
        s_last_ref_yaw = desired_yaw;
        s_ref_yaw_initialized = true;
    }

    double dt = 0.005;
    if (!s_prev_stamp.isZero())
    {
        dt = (msg->header.stamp - s_prev_stamp).toSec();
        dt = clampValue(dt, 0.001, 0.05);
    }
    s_prev_stamp = msg->header.stamp;

    const double max_dyaw = kYawSlewRate * dt;
    const double yaw_err_cmd = wrapToPi(desired_yaw - s_last_ref_yaw);
    s_last_ref_yaw = wrapToPi(s_last_ref_yaw + clampValue(yaw_err_cmd, -max_dyaw, max_dyaw));

    const std::vector<QuadrotorLinearMPC::State> ref_traj = buildReferenceTrajectory(X_real, target, s_last_ref_yaw);
    const MPCResult mpc_result = g_mpc_controller->solve(X_real, ref_traj);

    airsim_ros::RotorPWM pwm_msg;
    pwm_msg.header.stamp = msg->header.stamp;
    pwm_msg.rotorPWM0 = mpc_result.rotor_pwm[0];
    pwm_msg.rotorPWM1 = mpc_result.rotor_pwm[1];
    pwm_msg.rotorPWM2 = mpc_result.rotor_pwm[2];
    pwm_msg.rotorPWM3 = mpc_result.rotor_pwm[3];
    g_pwm_publisher.publish(pwm_msg);

    ROS_INFO_THROTTLE(
        0.5,
        "MPC wp=%d/%zu cur=(%.2f %.2f %.2f) tgt=(%.2f %.2f %.2f) err=(%.2f %.2f %.2f) "
        "u=[T %.2f tx %.2f ty %.2f tz %.2f] pwm=[%.2f %.2f %.2f %.2f] J=%.2f",
        current_wp_idx,
        spline_path.empty() ? 0 : (spline_path.size() - 1),
        cur_pos.x(),
        cur_pos.y(),
        cur_pos.z(),
        target.x(),
        target.y(),
        target.z(),
        pos_err.x(),
        pos_err.y(),
        pos_err.z(),
        mpc_result.u[0],
        mpc_result.u[1],
        mpc_result.u[2],
        mpc_result.u[3],
        mpc_result.rotor_pwm[0],
        mpc_result.rotor_pwm[1],
        mpc_result.rotor_pwm[2],
        mpc_result.rotor_pwm[3],
        mpc_result.final_cost);
}