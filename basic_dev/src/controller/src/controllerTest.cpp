#include "controllerTest.hpp"
#include <fstream>
#include <sstream>
#include <cmath>

bool loadSplines(const std::string& filepath,
                 std::vector<std::vector<Eigen::Vector3d>>& paths)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open Splines.txt: %s", filepath.c_str());
        return false;
    }
    paths.clear();
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<Eigen::Vector3d> path;
        double x, y, z;
        while (iss >> x >> y >> z) {
            path.emplace_back(x, y, z);
        }
        if (path.size() >= 4) {
            paths.push_back(path);
            ROS_INFO("Loaded path with %zu waypoints, start=(%.1f,%.1f,%.1f)",
                     path.size(), path[0].x(), path[0].y(), path[0].z());
        }
    }
    ROS_INFO("Total %zu paths loaded", paths.size());
    return !paths.empty();
}

bool build_stage1_path_from_start(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_fwd_ned)
{
    if (globalPaths.empty()) return false;
    double best_d = 1e9;
    int best_path = -1;
    int best_idx = -1;
    for (int k = 0; k < (int)globalPaths.size(); ++k) {
        const auto& p = globalPaths[k];
        if (p.size() < 4) continue;
        for (int i = 0; i < (int)p.size(); ++i) {
            const double d = (p[i] - start_pos).norm();
            if (d < best_d) {
                best_d = d;
                best_path = k;
                best_idx = i;
            }
        }
    }
    if (best_path < 0 || best_idx < 0) return false;
    const auto& src = globalPaths[best_path];
    globalPath.clear();

    // 按起飞朝向选路径方向，避免“选中反向片段”导致过门后折返
    Eigen::Vector2d fwd_dir(start_fwd_ned.x(), start_fwd_ned.y());
    if (fwd_dir.norm() < 1e-6) fwd_dir = Eigen::Vector2d(1.0, 0.0);
    fwd_dir.normalize();

    Eigen::Vector2d tan_forward(0.0, 0.0), tan_reverse(0.0, 0.0);
    if (best_idx < (int)src.size() - 1) {
        tan_forward = Eigen::Vector2d(src[best_idx + 1].x() - src[best_idx].x(),
                                      src[best_idx + 1].y() - src[best_idx].y());
    }
    if (best_idx > 0) {
        tan_reverse = Eigen::Vector2d(src[best_idx - 1].x() - src[best_idx].x(),
                                      src[best_idx - 1].y() - src[best_idx].y());
    }
    if (tan_forward.norm() > 1e-6) tan_forward.normalize();
    if (tan_reverse.norm() > 1e-6) tan_reverse.normalize();
    const double score_forward = fwd_dir.dot(tan_forward);
    const double score_reverse = fwd_dir.dot(tan_reverse);
    const bool use_forward = (score_forward >= score_reverse);

    if (use_forward) {
        const int end_i = std::min((int)src.size() - 1, best_idx + g_stage1_waypoint_count - 1);
        for (int i = best_idx; i <= end_i; ++i) globalPath.push_back(src[i]);
    } else {
        const int end_i = std::max(0, best_idx - g_stage1_waypoint_count + 1);
        for (int i = best_idx; i >= end_i; --i) globalPath.push_back(src[i]);
    }

    if (globalPath.size() < 8) return false;
    Pwend = globalPath.back();
    next_goal_index = 0;
    get_end_goal = true;
    g_stage1_path_is_provisional = true;
    ROS_WARN("Stage1 auto path: path=%d start_idx=%d len=%zu nearest_dist=%.2f dir=%s score_f=%.2f score_r=%.2f",
             best_path, best_idx, globalPath.size(), best_d, use_forward ? "forward" : "reverse",
             score_forward, score_reverse);
    ROS_WARN("  start=(%.1f,%.1f,%.1f) end=(%.1f,%.1f,%.1f)",
             globalPath.front().x(), globalPath.front().y(), globalPath.front().z(),
             globalPath.back().x(), globalPath.back().y(), globalPath.back().z());
    return true;
}

bool build_path_from_start_end(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos)
{
    double best_cost = 1e9;
    int best_idx = -1, best_start_i = -1, best_end_i = -1;
    for (int k = 0; k < (int)globalPaths.size(); k++) {
        if (globalPaths[k].size() < 8) continue;
        int i_start = -1, i_end = -1;
        double d_start = 1e9, d_end = 1e9;
        for (int i = 0; i < (int)globalPaths[k].size(); ++i) {
            const double ds = (globalPaths[k][i] - start_pos).norm();
            if (ds < d_start) { d_start = ds; i_start = i; }
            const double de = (globalPaths[k][i] - end_pos).norm();
            if (de < d_end) { d_end = de; i_end = i; }
        }
        if (i_start < 0 || i_end < 0) continue;
        const int seg_len = std::abs(i_end - i_start) + 1;
        double cost = d_start + d_end;
        if (seg_len < 8) cost += 30.0;
        if (d_start > 120.0 || d_end > 120.0) cost += 200.0;
        if (cost < best_cost) {
            best_cost = cost;
            best_idx = k;
            best_start_i = i_start;
            best_end_i = i_end;
        }
    }

    globalPath.clear();
    if (best_idx < 0 || best_start_i < 0 || best_end_i < 0) return false;
    if (best_start_i <= best_end_i) {
        for (int i = best_start_i; i <= best_end_i; ++i) {
            globalPath.push_back(globalPaths[best_idx][i]);
        }
    } else {
        for (int i = best_start_i; i >= best_end_i; --i) {
            globalPath.push_back(globalPaths[best_idx][i]);
        }
    }
    if (globalPath.size() < 8) return false;

    next_goal_index = 0;
    get_end_goal = true;
    g_stage1_path_is_provisional = false;
    ROS_WARN("Path matched by end_goal: path=%d len=%zu cost=%.2f start_idx=%d end_idx=%d",
             best_idx, globalPath.size(), best_cost, best_start_i, best_end_i);
    ROS_WARN("  matched start=(%.1f,%.1f,%.1f) end=(%.1f,%.1f,%.1f)",
             globalPath.front().x(), globalPath.front().y(), globalPath.front().z(),
             globalPath.back().x(), globalPath.back().y(), globalPath.back().z());
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_test");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string splines_path;
    n.param<std::string>("splines_path", splines_path,
        "/home/bai/rmua2026/IntelligentUAVChampionshipBase/basic_dev/src/controller/src/Splines.txt");
    if (!loadSplines(splines_path, globalPaths)) {
        ROS_ERROR("No valid spline path loaded, aborting.");
        return -1;
    }
    pn.param<float>("pwm_slew_step", g_pwm_slew_step, 0.02f);
    pn.param<bool>("stage1_only", g_stage1_only, true);
    pn.param<bool>("auto_path_on_init", g_auto_path_on_init, true);
    pn.param<int>("stage1_waypoint_count", g_stage1_waypoint_count, 120);
    ROS_INFO("Config: slew=%.4f stage1=%d auto_path=%d wp_count=%d",
             g_pwm_slew_step, (int)g_stage1_only, (int)g_auto_path_on_init, g_stage1_waypoint_count);

    g_takeoff_client = n.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    g_pwm_publisher = n.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);
    g_vel_publisher = n.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);
    g_vel_body_publisher = n.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_body_cmd", 1);
    ros::Subscriber odom_suber = n.subscribe<nav_msgs::Odometry>("/eskf_odom", 1, odom_cb);
    ros::Subscriber init_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, init_pose_cb);
    ros::Subscriber end_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_goal", 1, end_position_cb);

    airsim_ros::Takeoff tf_cmd;
    tf_cmd.request.waitOnLastTask = 1;
    g_takeoff_client.call(tf_cmd);

    ros::Rate loop_rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaternion Q(msg->pose.orientation.w, msg->pose.orientation.x,
                        msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rotationM = Q.normalized().toRotationMatrix();
    Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Tw0 = Eigen::Matrix4d::Identity();
    Tw0.block(0, 0, 3, 3) = rotationM;
    Tw0.block(0, 3, 3, 1) = pos;
    Twb_last = Tw0;
    get_init_pose = true;
    if (g_stage1_only && g_auto_path_on_init && !get_end_goal) {
        const Eigen::Vector3d start_pos = Tw0.block<3,1>(0, 3);
        Eigen::Vector3d start_fwd_ned = Tw0.block<3,3>(0,0).col(0);
        if (!build_stage1_path_from_start(start_pos, start_fwd_ned)) {
            ROS_ERROR("Stage1 auto path selection failed.");
        }
    }
}

void end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static bool has_last_end = false;
    static Eigen::Vector3d last_end = Eigen::Vector3d::Zero();
    Pwend = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (get_init_pose) {
        Eigen::Vector3d start_pos = Tw0.block<3,1>(0, 3);
        const bool end_changed = (!has_last_end) || ((Pwend - last_end).norm() > 1.0);
        const bool need_update = (!get_end_goal || g_stage1_path_is_provisional || end_changed);
        if (need_update) {
            ROS_INFO("Matching path: start=(%.1f,%.1f,%.1f) end=(%.1f,%.1f,%.1f)",
                     start_pos.x(), start_pos.y(), start_pos.z(),
                     Pwend.x(), Pwend.y(), Pwend.z());
            if (!build_path_from_start_end(start_pos, Pwend)) {
                ROS_ERROR("Failed to match path using end_goal.");
            }
            last_end = Pwend;
            has_last_end = true;
        }
    }
}

// ============================================================================
// 核心控制回调：干净版，无多余安全层，信赖 PD 控制器 + 直接电机映射
// ============================================================================
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cb_cnt++;

    auto clamp01 = [](float v) -> float { return std::max(0.0f, std::min(1.0f, v)); };
    auto clampf = [](float v, float lo, float hi) -> float { return std::max(lo, std::min(hi, v)); };

    // 悬停 PWM: mass*g/(4*Fmax) = 0.9*9.8/(4*12.538) ≈ 0.176
    const float HOVER_PWM = 0.18f;

    auto publish_hover = [&]() {
        if (!g_pwm_initialized) {
            g_last_pwm = Eigen::Vector4f::Constant(HOVER_PWM);
            g_pwm_initialized = true;
        }
        for (int i = 0; i < 4; ++i) {
            float delta = clampf(HOVER_PWM - g_last_pwm[i], -g_pwm_slew_step, g_pwm_slew_step);
            g_last_pwm[i] = clamp01(g_last_pwm[i] + delta);
        }
        airsim_ros::RotorPWM cmd;
        cmd.rotorPWM0 = g_last_pwm[0];
        cmd.rotorPWM1 = g_last_pwm[1];
        cmd.rotorPWM2 = g_last_pwm[2];
        cmd.rotorPWM3 = g_last_pwm[3];
        g_pwm_publisher.publish(cmd);
    };

    // 等待 ESKF 收敛 + 路径就绪
    if (cb_cnt < 400 || !get_init_pose || !get_end_goal || globalPath.size() < 8) {
        publish_hover();
        return;
    }

    // === 1. 解析当前状态 (NED 世界系) ===
    Eigen::Quaterniond Q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twb.block<3,3>(0, 0) = Q.normalized().toRotationMatrix();
    Twb(0, 3) = msg->pose.pose.position.x;
    Twb(1, 3) = msg->pose.pose.position.y;
    Twb(2, 3) = msg->pose.pose.position.z;

    // === 2. NED → FLU 坐标变换 ===
    Eigen::Matrix4d TWfluWned;
    TWfluWned << 1,0,0,0, 0,-1,0,0, 0,0,-1,0, 0,0,0,1;
    Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();
    Eigen::Matrix4d TWflub = TWfluWned * Twb * TWfluWned.inverse();
    Eigen::Matrix4d T0flub = TWflu0.inverse() * TWflub;

    Eigen::Vector3d VWned(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
    // X_real 位置在 start-FLU 系，速度也必须在同一坐标系，否则偏航时 PD 速度反馈失配
    Eigen::Vector3d VWflu = TWfluWned.block<3,3>(0,0) * VWned;
    Eigen::Vector3d V0flu = TWflu0.block<3,3>(0,0).transpose() * VWflu;
    Eigen::Vector3d Wned(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                          msg->twist.twist.angular.z);
    Eigen::Vector3d Wflu = TWfluWned.block<3,3>(0,0) * Wned;

    const double sin_arg = std::max(-1.0, std::min(1.0, T0flub(2, 1)));
    const float phi   = (float)std::asin(sin_arg);
    const float cos_phi = std::cos(phi);
    const float theta = (std::abs(cos_phi) > 1e-6f) ?
        (float)std::atan2(-T0flub(2,0)/cos_phi, T0flub(2,2)/cos_phi) : 0.0f;
    const float psi   = (std::abs(cos_phi) > 1e-6f) ?
        (float)std::atan2(-T0flub(0,1)/cos_phi, T0flub(1,1)/cos_phi) : 0.0f;

    Eigen::VectorXf X_real(12);
    X_real << (float)T0flub(0,3), (float)T0flub(1,3), (float)T0flub(2,3),
              (float)V0flu.x(), (float)V0flu.y(), (float)V0flu.z(),
              phi, theta, psi, (float)Wflu.x(), (float)Wflu.y(), (float)Wflu.z();

    // === 3. Pure Pursuit 路径跟踪 (2D 水平距离，不受高度差干扰) ===
    Eigen::Vector3d cur_NED(Twb(0,3), Twb(1,3), Twb(2,3));

    // 用“线段投影进度”替代“离散最近点”：
    // 1) 对附近路径段做投影求最近点
    // 2) 进度做单调不减，彻底避免折返追身后点
    double best_d2 = 1e18;
    int best_seg = std::min(next_goal_index, (int)globalPath.size() - 2);
    double best_t = 0.0;
    const int seg_start = std::max(0, next_goal_index - 2);
    const int seg_end = std::min((int)globalPath.size() - 2, next_goal_index + 120);
    for (int i = seg_start; i <= seg_end; ++i) {
        const double x0 = globalPath[i].x();
        const double y0 = globalPath[i].y();
        const double x1 = globalPath[i + 1].x();
        const double y1 = globalPath[i + 1].y();
        const double vx = x1 - x0;
        const double vy = y1 - y0;
        const double wx = cur_NED.x() - x0;
        const double wy = cur_NED.y() - y0;
        const double vv = vx * vx + vy * vy;
        const double t = (vv > 1e-6) ? std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / vv)) : 0.0;
        const double px = x0 + t * vx;
        const double py = y0 + t * vy;
        const double dx = cur_NED.x() - px;
        const double dy = cur_NED.y() - py;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_seg = i;
            best_t = t;
        }
    }
    const double progress_raw = (double)best_seg + best_t;
    static bool progress_inited = false;
    static double progress_mono = 0.0;
    if (!progress_inited) {
        progress_mono = progress_raw;
        progress_inited = true;
    } else {
        progress_mono = std::max(progress_mono, progress_raw);
    }
    next_goal_index = std::min((int)globalPath.size() - 2, (int)std::floor(progress_mono));
    const double min_dist = std::sqrt(best_d2);

    int lookahead = 6;
    int target_idx = std::min(next_goal_index + lookahead, (int)globalPath.size() - 1);

    Eigen::Vector3d target_NED = globalPath[target_idx];

    ROS_INFO_THROTTLE(1.0,
        "Track idx=%d/%zu | cur_NED=(%.1f,%.1f,%.1f) tgt_NED=(%.1f,%.1f,%.1f) dist=%.1f",
        target_idx, globalPath.size(),
        cur_NED.x(), cur_NED.y(), cur_NED.z(),
        target_NED.x(), target_NED.y(), target_NED.z(), min_dist);

    // === 4. 目标点转到起点 FLU 系 ===
    Eigen::Vector4d target_homo(target_NED.x(), target_NED.y(), target_NED.z(), 1.0);
    Eigen::Vector4d target_FLU = TWflu0.inverse() * TWfluWned * target_homo;

    static float cruise_alt = 0.0f;
    static bool cruise_alt_set = false;
    if (!cruise_alt_set) {
        cruise_alt = X_real[2];
        cruise_alt_set = true;
        ROS_WARN("Cruise altitude locked: z_FLU = %.2f m (drone hovering here after takeoff)", cruise_alt);
    }

    float des_x = (float)target_FLU(0);
    float des_y = (float)target_FLU(1);
    float des_z = cruise_alt;

    des_x = clampf(des_x, X_real[0] - 5.0f, X_real[0] + 5.0f);
    des_y = clampf(des_y, X_real[1] - 5.0f, X_real[1] + 5.0f);

    float yaw_dx = des_x - X_real[0];
    float yaw_dy = des_y - X_real[1];
    float target_yaw = (std::abs(yaw_dx) > 0.05f || std::abs(yaw_dy) > 0.05f) ?
        std::atan2(yaw_dy, yaw_dx) : psi;

    Eigen::VectorXf X_des(12);
    X_des << des_x, des_y, des_z, 0,0,0, 0,0,target_yaw, 0,0,0;

    // === 5. PD 控制 ===
    Eigen::Vector4f output = g_PDcontroller.execute(X_des, X_real);

    // === 5b. 姿态保护：只削减推力（均值），保留姿态纠偏力矩（电机差异） ===
    //   旧实现把 4 个电机都拉到 HOVER_PWM，会把纠偏力矩清零→无法恢复→翻转坠毁
    const float TILT_SOFT = 0.60f;
    const float TILT_HARD = 1.00f;
    float max_tilt = std::max(std::abs(phi), std::abs(theta));
    if (max_tilt > TILT_SOFT) {
        float blend = std::min(1.0f, (max_tilt - TILT_SOFT) / (TILT_HARD - TILT_SOFT));
        float avg = (output[0] + output[1] + output[2] + output[3]) * 0.25f;
        float target_avg = (max_tilt < 1.57f)
            ? avg * (1.0f - blend) + HOVER_PWM * blend
            : 0.10f;
        float shift = target_avg - avg;
        for (int i = 0; i < 4; ++i) {
            output[i] = std::max(0.0f, std::min(1.0f, output[i] + shift));
        }
        ROS_WARN_THROTTLE(0.5, "TILT PROTECT: tilt=%.2f blend=%.1f avg=%.3f", max_tilt, blend, target_avg);
    }

    // === 6. 直接映射 output[i] → rotorPWM[i] + slew rate ===
    if (!g_pwm_initialized) {
        for (int i = 0; i < 4; ++i) g_last_pwm[i] = clamp01(output[i]);
        g_pwm_initialized = true;
    }
    for (int i = 0; i < 4; ++i) {
        float target_pwm = clamp01(output[i]);
        float delta = clampf(target_pwm - g_last_pwm[i], -g_pwm_slew_step, g_pwm_slew_step);
        g_last_pwm[i] = clamp01(g_last_pwm[i] + delta);
    }

    airsim_ros::RotorPWM pwm_cmd;
    pwm_cmd.rotorPWM0 = g_last_pwm[0];
    pwm_cmd.rotorPWM1 = g_last_pwm[1];
    pwm_cmd.rotorPWM2 = g_last_pwm[2];
    pwm_cmd.rotorPWM3 = g_last_pwm[3];
    g_pwm_publisher.publish(pwm_cmd);

    if (cb_cnt % 200 == 0) {
        ROS_INFO("PWM[%.3f %.3f %.3f %.3f] pos=(%.1f,%.1f,%.1f) des=(%.1f,%.1f,%.1f) phi=%.2f th=%.2f psi=%.2f WP%d/%zu d=%.1f",
                 g_last_pwm[0], g_last_pwm[1], g_last_pwm[2], g_last_pwm[3],
                 X_real[0], X_real[1], X_real[2], des_x, des_y, des_z,
                 phi, theta, psi, target_idx, globalPath.size(), min_dist);
    }
}
