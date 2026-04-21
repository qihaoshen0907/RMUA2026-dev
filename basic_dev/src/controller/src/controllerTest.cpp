#include "controllerTest.hpp"
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>

std::vector<Eigen::Vector3d> spline_path;
int current_wp_idx = 0;
bool spline_loaded = false;
bool g_path_complete = false;

namespace
{
constexpr double kTakeoffHoldSec       = 3.0;

// 原来的单一距离阈值保留成兼容参数
constexpr double kWaypointReachDist    = 0.8;

// 新增：分开控制 waypoint 到达容差
constexpr double kWaypointReachXY      = 1.0;   // 平面内 1m 内算到
constexpr double kWaypointReachZ       = 0.35;  // 高度误差 0.35m 内算到
constexpr double kWaypointSkipBehindX  = 0.25;  // 若 x 已经飞过这个点 0.25m，也允许跳点
constexpr double kWaypointSkipXY       = 1.2;   // 跳点时的平面容差
constexpr double kWaypointSkipZ        = 0.5;   // 跳点时的高度容差

constexpr double kCruiseSpeed          = 2.0;
constexpr double kMinSpeed             = 0.30;
constexpr double kVelGain              = 0.8;
constexpr double kDensifySegLen        = 2.0;
constexpr double kMaxCmdVx             = 2.0;
constexpr double kMaxCmdVy             = 0.0;   // 仍然禁用横向速度
constexpr double kMaxCmdVz             = 1.0;
constexpr double kYawRateP             = 1.5;
constexpr double kYawRateMax           = 1.0;
constexpr double kHeadingSlowCosFloor  = 0.15;

int s_vel_cmd_va = 1;
double s_waypoint_reach_dist = kWaypointReachDist;

// 新增参数：允许通过 rosparam 调
double s_waypoint_reach_xy = kWaypointReachXY;
double s_waypoint_reach_z = kWaypointReachZ;
double s_waypoint_skip_behind_x = kWaypointSkipBehindX;
double s_waypoint_skip_xy = kWaypointSkipXY;
double s_waypoint_skip_z = kWaypointSkipZ;

double s_cruise_speed = kCruiseSpeed;
double s_min_speed = kMinSpeed;
double s_vel_gain = kVelGain;
double s_max_cmd_vx = kMaxCmdVx;
double s_max_cmd_vy = kMaxCmdVy;
double s_max_cmd_vz = kMaxCmdVz;
double s_yaw_rate_p = kYawRateP;
double s_yaw_rate_max = kYawRateMax;

template <typename T>
T clampValue(T v, T lo, T hi)
{
    return std::max(lo, std::min(hi, v));
}

double wrapToPi(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
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
    // 适合当前这类“主要沿 x 正方向往前飞”的 spline
    const double dx = target.x() - cur_pos.x();
    const double dy = target.y() - cur_pos.y();
    const double dz = target.z() - cur_pos.z();

    const double xy_err = std::sqrt(dx * dx + dy * dy);
    const double z_err = std::abs(dz);

    // 如果这个点已经被“飞过去”了，并且横向/高度误差也还在可接受范围内，就跳过
    return (dx < -behind_x_tol && xy_err <= xy_tol && z_err <= z_tol);
}
} // namespace

std::vector<Eigen::Vector3d> densifyPath(const std::vector<Eigen::Vector3d>& path, double max_seg_len)
{
    std::vector<Eigen::Vector3d> refined_path;

    if (path.empty()) return refined_path;
    if (path.size() == 1)
    {
        refined_path.push_back(path[0]);
        return refined_path;
    }

    for (int i = 0; i < static_cast<int>(path.size()) - 1; i++)
    {
        Eigen::Vector3d p0 = path[i];
        Eigen::Vector3d p1 = path[i + 1];

        refined_path.push_back(p0);

        Eigen::Vector3d diff = p1 - p0;
        double dist = diff.norm();

        if (dist > max_seg_len)
        {
            int num_segments = static_cast<int>(std::ceil(dist / max_seg_len));
            for (int k = 1; k < num_segments; k++)
            {
                double alpha = static_cast<double>(k) / static_cast<double>(num_segments);
                Eigen::Vector3d mid = p0 + alpha * diff;
                refined_path.push_back(mid);
            }
        }
    }

    refined_path.push_back(path.back());
    return refined_path;
}

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
    TWfluWned << 1, 0, 0, 0,
                 0,-1, 0, 0,
                 0, 0,-1, 0,
                 0, 0, 0, 1;

    Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();

    double x, y, z;
    int raw_idx = 0;
    while (file >> x >> y >> z)
    {
        Eigen::Vector4d p_world_ned(x, y, z, 1.0);
        Eigen::Vector4d p_world_flu = TWfluWned * p_world_ned;
        Eigen::Vector4d p_local_flu = TWflu0.inverse() * p_world_flu;

        spline_path.emplace_back(p_local_flu.x(), p_local_flu.y(), p_local_flu.z());

        if (raw_idx < 10)
        {
            ROS_INFO("Spline raw[%d] world_ned=(%.3f %.3f %.3f) -> local_flu=(%.3f %.3f %.3f)",
                     raw_idx, x, y, z,
                     p_local_flu.x(), p_local_flu.y(), p_local_flu.z());
        }
        raw_idx++;
    }

    file.close();

    ROS_INFO("Loaded raw spline points: %zu", spline_path.size());

    if (!spline_path.empty())
    {
        spline_path = densifyPath(spline_path, kDensifySegLen);
        spline_loaded = true;
        ROS_INFO("Spline converted to local frame and densified. Total points: %zu", spline_path.size());
    }
    else
    {
        ROS_ERROR("Spline file is empty.");
    }
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

    const std::string default_spline =
        "src/controller/src/baseline.txt";

    pn.param<std::string>("spline_path", g_spline_file_path, default_spline);
    pn.param("vel_cmd_va", s_vel_cmd_va, 1);

    // 旧参数兼容
    pn.param("waypoint_reach_dist", s_waypoint_reach_dist, kWaypointReachDist);

    // 新参数
    pn.param("waypoint_reach_xy", s_waypoint_reach_xy, kWaypointReachXY);
    pn.param("waypoint_reach_z", s_waypoint_reach_z, kWaypointReachZ);
    pn.param("waypoint_skip_behind_x", s_waypoint_skip_behind_x, kWaypointSkipBehindX);
    pn.param("waypoint_skip_xy", s_waypoint_skip_xy, kWaypointSkipXY);
    pn.param("waypoint_skip_z", s_waypoint_skip_z, kWaypointSkipZ);

    pn.param("cruise_speed", s_cruise_speed, kCruiseSpeed);
    pn.param("min_speed", s_min_speed, kMinSpeed);
    pn.param("vel_gain", s_vel_gain, kVelGain);
    pn.param("max_cmd_vx", s_max_cmd_vx, kMaxCmdVx);
    pn.param("max_cmd_vy", s_max_cmd_vy, kMaxCmdVy);
    pn.param("max_cmd_vz", s_max_cmd_vz, kMaxCmdVz);
    pn.param("yaw_rate_p", s_yaw_rate_p, kYawRateP);
    pn.param("yaw_rate_max", s_yaw_rate_max, kYawRateMax);

    g_takeoff_client = n.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    g_vel_publisher  = n.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_body_cmd", 1);

    ros::Subscriber odom_suber =
        n.subscribe<nav_msgs::Odometry>("/eskf_odom", 1, odom_cb);
    ros::Subscriber init_pose_suber =
        n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, init_pose_cb);
    ros::Subscriber end_pose_suber =
        n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_goal", 1, end_position_cb);

    ROS_INFO("controller_test started.");
    ROS_INFO("spline_path = %s", g_spline_file_path.c_str());
    ROS_INFO("mode = takeoff -> hold %.1f s -> load global spline -> convert to local -> forward+yaw tracking",
             kTakeoffHoldSec);
    ROS_INFO("waypoint tolerance: xy=%.2f z=%.2f | skip: behind_x=%.2f xy=%.2f z=%.2f",
             s_waypoint_reach_xy, s_waypoint_reach_z,
             s_waypoint_skip_behind_x, s_waypoint_skip_xy, s_waypoint_skip_z);

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaternion Q(msg->pose.orientation.w,
                        msg->pose.orientation.x,
                        msg->pose.orientation.y,
                        msg->pose.orientation.z);
    Eigen::Matrix3d rotationM = Q.normalized().toRotationMatrix();
    Eigen::Vector3d pos(msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z);

    Tw0 = Eigen::Matrix4d::Identity();
    Tw0.block(0, 0, 3, 3) = rotationM;
    Tw0.block(0, 3, 3, 1) = pos;
    Twb_last = Tw0;
    get_init_pose = true;

    ROS_INFO_ONCE("Initial pose received.");
}

void end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Pwend = Eigen::Vector3d(msg->pose.position.x,
                            msg->pose.position.y,
                            msg->pose.position.z);

    if (get_init_pose && !get_end_goal)
    {
        for (auto ps : globalPaths)
        {
            if ((ps[0] - Tw0.block(0, 3, 3, 1)).norm() < 10)
            {
                for (int i = 0; i < static_cast<int>(ps.size()); i++)
                {
                    globalPath.emplace_back(ps[i]);
                }
                break;
            }
        }

        for (auto ps : globalPaths)
        {
            if ((ps[0] - Pwend).norm() < 10)
            {
                for (int i = 0; i < static_cast<int>(ps.size()); i++)
                {
                    globalPath.emplace_back(ps[ps.size() - 1 - i]);
                }
                break;
            }
        }

        get_end_goal = true;
    }
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!get_init_pose) return;

    Eigen::Quaternion Q(msg->pose.pose.orientation.w,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z);

    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twb.block(0, 0, 3, 3) = Q.normalized().toRotationMatrix();
    Twb(0, 3) = msg->pose.pose.position.x;
    Twb(1, 3) = msg->pose.pose.position.y;
    Twb(2, 3) = msg->pose.pose.position.z;

    Eigen::VectorXf X_real;
    X_real.resize(12);

    // 保留原始代码的坐标转换逻辑
    Eigen::Matrix4d TWfluWned;
    TWfluWned << 1, 0, 0, 0,
                 0,-1, 0, 0,
                 0, 0,-1, 0,
                 0, 0, 0, 1;

    Eigen::Matrix4d TWflu0  = TWfluWned * Tw0 * TWfluWned.inverse();
    Eigen::Matrix4d TWflub  = TWfluWned * Twb * TWfluWned.inverse();
    Eigen::Matrix4d T0flub  = TWflu0.inverse() * TWflub;

    Eigen::Vector3d VWned(msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);

    Eigen::Vector3d VBned = Twb.block(0, 0, 3, 3).inverse() * VWned;
    Eigen::Vector3d VBflu = TWfluWned.block<3, 3>(0, 0) * VBned;

    Eigen::Vector3d Wned(msg->twist.twist.angular.x,
                         msg->twist.twist.angular.y,
                         msg->twist.twist.angular.z);

    Eigen::Vector3d Wflu = TWfluWned.block<3, 3>(0, 0) * Wned;

    const float phi   = std::asin(T0flub(2, 1));
    const float theta = std::atan2(-T0flub(2, 0) / std::cos(phi),
                                   T0flub(2, 2) / std::cos(phi));
    const float psi   = std::atan2(-T0flub(0, 1) / std::cos(phi),
                                   T0flub(1, 1) / std::cos(phi));

    X_real << T0flub(0, 3), T0flub(1, 3), T0flub(2, 3),
              VBflu.x(), VBflu.y(), VBflu.z(),
              phi, theta, psi,
              Wflu.x(), Wflu.y(), Wflu.z();

    // takeoff -> hold 3 sec -> load spline
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
        return;
    }

    if (g_takeoff_sent && !spline_loaded)
    {
        if ((ros::Time::now() - g_takeoff_time).toSec() >= kTakeoffHoldSec)
        {
            loadSpline(g_spline_file_path);
        }
    }

    airsim_ros::VelCmd vel_cmd;
    vel_cmd.header.stamp = msg->header.stamp;
    vel_cmd.vx = 0.0;
    vel_cmd.vy = 0.0;
    vel_cmd.vz = 0.0;
    vel_cmd.yawRate = 0.0;
    vel_cmd.va = static_cast<uint8_t>(clampValue(s_vel_cmd_va, 0, 255));
    vel_cmd.stop = 0;

    if (!spline_loaded || spline_path.empty() || g_path_complete)
    {
        g_vel_publisher.publish(vel_cmd);
        return;
    }

    Eigen::Vector3d cur_pos(X_real[0], X_real[1], X_real[2]);

    // 连续跳点：只要当前点已经“足够接近”或者“已经飞过去且误差可接受”，就跳下一个
    while (current_wp_idx < static_cast<int>(spline_path.size()) - 1)
    {
        const Eigen::Vector3d& wp = spline_path[current_wp_idx];

        const bool reached = isWaypointReached(cur_pos, wp,
                                               s_waypoint_reach_xy,
                                               s_waypoint_reach_z);

        const bool skipped = shouldSkipWaypoint(cur_pos, wp,
                                                s_waypoint_skip_behind_x,
                                                s_waypoint_skip_xy,
                                                s_waypoint_skip_z);

        if (reached || skipped)
        {
            ROS_INFO("Advance waypoint %d -> %d (%s)",
                     current_wp_idx,
                     current_wp_idx + 1,
                     reached ? "reached" : "skipped");
            current_wp_idx++;
        }
        else
        {
            break;
        }
    }

    if (current_wp_idx >= static_cast<int>(spline_path.size()))
    {
        g_path_complete = true;
        ROS_INFO("Finished all waypoints.");
        g_vel_publisher.publish(vel_cmd);
        return;
    }

    Eigen::Vector3d target = spline_path[current_wp_idx];
    Eigen::Vector3d err = target - cur_pos;

    const double dx = err.x();
    const double dy = err.y();
    const double dz = err.z();

    const double xy_err = std::sqrt(dx * dx + dy * dy);
    const double dist = err.norm();

    // 最后一层保险：到范围内就停/切
    if (isWaypointReached(cur_pos, target, s_waypoint_reach_xy, s_waypoint_reach_z))
    {
        if (current_wp_idx < static_cast<int>(spline_path.size()) - 1)
        {
            current_wp_idx++;
            target = spline_path[current_wp_idx];
            err = target - cur_pos;
        }
        else
        {
            g_path_complete = true;
            ROS_INFO("Finished all waypoints.");
            g_vel_publisher.publish(vel_cmd);
            return;
        }
    }

    if (dist > 1e-6)
    {
        const double desired_yaw = std::atan2(err.y(), err.x());
        const double yaw_err = wrapToPi(desired_yaw - psi);

        // yaw 控制
        vel_cmd.yawRate = clampValue(-s_yaw_rate_p * yaw_err,
                                     -s_yaw_rate_max,
                                     s_yaw_rate_max);

        // 偏航误差大时降速，但不让它完全停死
        const double base_speed = clampValue(s_vel_gain * dist, s_min_speed, s_cruise_speed);
        double heading_scale = std::max(kHeadingSlowCosFloor, std::cos(std::abs(yaw_err)));

        if (std::abs(yaw_err) > 0.6)
            heading_scale *= 0.5;
        if (std::abs(yaw_err) > 1.0)
            heading_scale *= 0.5;

        const double forward_speed = std::max(0.18, base_speed * heading_scale);

        // 以前完全禁用 vy 会卡在点旁边；这里给一点点小的横向修正，但限制很小
        const double small_vy = clampValue(-0.20 * dy, -0.25, 0.25);

        vel_cmd.vx = clampValue(forward_speed, 0.0, s_max_cmd_vx);
        vel_cmd.vy = (std::abs(dy) > s_waypoint_reach_xy * 0.6) ? small_vy : 0.0;
        vel_cmd.vz = clampValue(dz * 0.6, -s_max_cmd_vz, s_max_cmd_vz);
    }

    g_vel_publisher.publish(vel_cmd);

    ROS_INFO_THROTTLE(
        0.5,
        "wp=%d/%zu cur=(%.2f %.2f %.2f) tgt=(%.2f %.2f %.2f) err=(%.2f %.2f %.2f) xy_err=%.2f cmd=(%.2f %.2f %.2f) yawRate=%.2f psi=%.2f",
        current_wp_idx,
        spline_path.size() - 1,
        cur_pos.x(), cur_pos.y(), cur_pos.z(),
        target.x(), target.y(), target.z(),
        err.x(), err.y(), err.z(),
        xy_err,
        vel_cmd.vx, vel_cmd.vy, vel_cmd.vz,
        vel_cmd.yawRate,
        psi);
}