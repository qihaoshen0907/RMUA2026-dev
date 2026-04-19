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
constexpr double kTakeoffHoldSec      = 3.0;
constexpr double kWaypointReachDist   = 1.0;
constexpr double kCruiseSpeed         = 1.8;
constexpr double kMinSpeed            = 0.35;
constexpr double kVelGain             = 0.6;
constexpr double kDensifySegLen       = 2.0;
constexpr double kMaxCmdVx            = 1.8;
constexpr double kMaxCmdVy            = 1.2;
constexpr double kMaxCmdVz            = 0.8;
constexpr double kYawRateCmd          = 0.0;

int s_vel_cmd_va = 1;

template <typename T>
T clampValue(T v, T lo, T hi)
{
    return std::max(lo, std::min(hi, v));
}
}

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

    // 和 odom_cb 里保持一致：NED -> FLU
    Eigen::Matrix4d TWfluWned;
    TWfluWned << 1, 0, 0, 0,
                 0,-1, 0, 0,
                 0, 0,-1, 0,
                 0, 0, 0, 1;

    // 起点在 world FLU 中的位姿
    Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();

    double x, y, z;
    int raw_idx = 0;
    while (file >> x >> y >> z)
    {
        // 这些点来自 /debug/pose_gt，按 world/NED 点处理
        Eigen::Vector4d p_world_ned(x, y, z, 1.0);

        // 先转到 world FLU
        Eigen::Vector4d p_world_flu = TWfluWned * p_world_ned;

        // 再转到“起点局部 FLU”
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
        "/home/bai/rmua2026/RMUA2026-dev/basic_dev/src/controller/src/Splines_new.txt";
    pn.param<std::string>("spline_path", g_spline_file_path, default_spline);
    pn.param("vel_cmd_va", s_vel_cmd_va, 1);

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
    ROS_INFO("mode = takeoff -> hold %.1f s -> load global spline -> convert to local -> direct velocity tracking",
             kTakeoffHoldSec);

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

    // 现在 cur_pos 和 target 都在“起点局部 FLU”里
    Eigen::Vector3d cur_pos(X_real[0], X_real[1], X_real[2]);
    Eigen::Vector3d target = spline_path[current_wp_idx];

    Eigen::Vector3d err = target - cur_pos;
    double dist = err.norm();

    if (dist < kWaypointReachDist)
    {
        if (current_wp_idx < static_cast<int>(spline_path.size()) - 1)
        {
            current_wp_idx++;
            target = spline_path[current_wp_idx];
            err = target - cur_pos;
            dist = err.norm();

            ROS_INFO("Switch to waypoint %d / %zu: %.3f %.3f %.3f",
                     current_wp_idx,
                     spline_path.size() - 1,
                     target.x(), target.y(), target.z());
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
        Eigen::Vector3d dir = err.normalized();
        double speed = clampValue(kVelGain * dist, kMinSpeed, kCruiseSpeed);
        Eigen::Vector3d v_local = dir * speed;

        // 先直接发布 local 速度，不再做额外 body 旋转
        vel_cmd.vx = clampValue(v_local.x(), -kMaxCmdVx, kMaxCmdVx);
        vel_cmd.vy = clampValue(v_local.y(), -kMaxCmdVy, kMaxCmdVy);
        vel_cmd.vz = clampValue(v_local.z(), -kMaxCmdVz, kMaxCmdVz);
        vel_cmd.yawRate = kYawRateCmd;
    }

    g_vel_publisher.publish(vel_cmd);

    ROS_INFO_THROTTLE(
        0.5,
        "wp=%d/%zu cur=(%.2f %.2f %.2f) tgt=(%.2f %.2f %.2f) err=(%.2f %.2f %.2f) cmd=(%.2f %.2f %.2f) psi=%.2f",
        current_wp_idx,
        spline_path.size() - 1,
        cur_pos.x(), cur_pos.y(), cur_pos.z(),
        target.x(), target.y(), target.z(),
        err.x(), err.y(), err.z(),
        vel_cmd.vx, vel_cmd.vy, vel_cmd.vz,
        psi);
}