#include "PDcontroller.hpp"
#include <iostream>
#include <cmath>

Eigen::Vector4f UAVLinearController::execute(Eigen::VectorXf& X_des, Eigen::VectorXf& X_real)
{
    auto clampf = [](float v, float lo, float hi) -> float {
        return std::max(lo, std::min(hi, v));
    };
    auto wrap_pi = [](float a) -> float {
        while (a > static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
        while (a < -static_cast<float>(M_PI)) a += static_cast<float>(2.0 * M_PI);
        return a;
    };

    float cur_e_z = X_des[2]-X_real[2];
    float u1 = m_mass * m_gravity + m_mass*(m_Kdz * (cur_e_z-m_last_e_z) + m_Kpz * cur_e_z);
    // FLU 坐标系：vz<0 表示下降。当下降时增加推力，上升时减少推力。
    u1 -= m_mass * m_Kvz_damp * X_real[5];
    u1 *= 1.00f;
    m_last_e_z = cur_e_z;
    if(u1 > m_Fmax * 4)u1 = m_Fmax * 4;
    float cur_e_x = X_des[0] - X_real[0];
    float vxd = std::min<float>(m_max_v, m_Kdx * (cur_e_x - m_last_e_x) + m_Kpx * cur_e_x);
    vxd = std::max<float>(vxd, -m_max_v);
    m_last_e_x = cur_e_x;
    float cur_e_vx = vxd - X_real[3];
    float axd = std::min<float>(m_max_acc, m_Kdvx * (cur_e_vx - m_last_e_vx) + m_Kpvx * cur_e_vx);
    axd = std::max<float>(axd, -m_max_acc);
    m_last_e_vx = cur_e_vx;

    float cur_e_y = X_des[1] - X_real[1];
    float vyd = std::min<float>(m_max_v, m_Kdy * (cur_e_y - m_last_e_y) + m_Kpy * cur_e_y);
    vyd = std::max<float>(vyd, -m_max_v);
    m_last_e_y = cur_e_y;
    float cur_e_vy = vyd - X_real[4];
    float ayd = std::min<float>(m_max_acc, m_Kdvy * (cur_e_vy - m_last_e_vy) + m_Kpvy * cur_e_vy);
    ayd = std::max<float>(ayd, -m_max_acc);
    m_last_e_vy = cur_e_vy;

    // 力→姿态分解必须用 *实际* 航向，而非期望航向；否则偏航滞后时推力方向错误
    float actual_psi = X_real[8];
    float phid = std::min<float>(m_mass / u1 * (axd*sinf(actual_psi) - ayd*cosf(actual_psi)), m_max_ang);
    float thetad = std::min<float>(m_mass / u1 * (axd*cosf(actual_psi) + ayd*sinf(actual_psi)), m_max_ang);
    phid = std::max<float>(phid, -m_max_ang);
    thetad = std::max<float>(thetad, -m_max_ang);

    float cur_e_phi = phid - X_real[6];
    if(m_last_e_phi == 0)m_last_e_phi = cur_e_phi;
    // 力矩限幅收紧，避免姿态通道瞬时打满导致振荡/发散
    float torquex = m_Ixx * (m_Kpphi * cur_e_phi + m_Kdphi * (cur_e_phi - m_last_e_phi));
    torquex = clampf(torquex, -0.10f, 0.10f);
    m_last_e_phi = cur_e_phi;
    float cur_e_theta = (thetad - X_real[7]);
    if(m_last_e_theta == 0) m_last_e_theta = cur_e_theta;
    float torquey = m_Iyy * (m_Kptheta * cur_e_theta + m_Kdtheta * (cur_e_theta - m_last_e_theta));
    torquey = clampf(torquey, -0.10f, 0.10f);
    m_last_e_theta = cur_e_theta;
    // std::cout<<m_cnt<<", des z: "<<X_des[2]<<" , real z: "<< X_real[2]<<std::endl;
    // std::cout<<m_cnt<<", des x: "<<X_des[0]<<" , real x: "<< X_real[0]<<
    //     ", des vx: "<< vxd<<", real vx: "<<X_real[3]<<", des ax: "<<axd <<", des thata: "<<thetad<<", real theta: "<<X_real[7]
    //     <<", ty: "<<torquey<< std::endl;
    // std::cout<<m_cnt<<", des y: "<<X_des[1]<<" , real y: "<< X_real[1]<<
    //     ", des vy: "<< vyd<<", real vy: "<<X_real[4]<<", des ay: "<<ayd <<", des phi: "<< phid<<", real phi: "<<X_real[6]
    //     <<", tx: "<<torquex<< std::endl;
    m_cnt ++;
    // 关键修复：偏航误差必须做角度包裹，否则跨 ±pi 会瞬间跳变到 ~6.28，引发发散
    float cur_e_psi = wrap_pi(X_des[8] - X_real[8]);
    float torquez = m_Izz * (m_Kppsi *  cur_e_psi + m_Kdpsi * (cur_e_psi - m_last_e_psi));
    torquez = clampf(torquez, -0.06f, 0.06f);
    m_last_e_psi = cur_e_psi;
    Eigen::Matrix4f M;
    M << 1, 1, 1, 1,
     -1, 1, 1, -1, 
     -1, 1, -1, 1, 
     -1, -1, 1, 1;
    Eigen::Vector4f u = Eigen::Vector4f(u1/m_Ct, torquex/(m_armlength*m_Ct*0.707), torquey/(m_armlength*m_Ct*0.707), torquez/m_Cq);
    Eigen::Vector4f thrust = m_Ct * M.inverse() * u;

    // 将推力归一化到 [0, 1] 的“PWM 控制量”。
    // 重要：不要逐个电机 hard-clamp 到 pwm_min(0.1)，否则会导致某些电机长期饱和，
    // 从而造成姿态震荡/“跳舞”。这里改成：若最小电机低于 pwm_min，就对四个电机整体
    // 加偏置（shift），保持相对差异。
    const float pwm_min = 0.1f;

    Eigen::Vector4f pwm = thrust / m_Fmax;

    // NaN/Inf 防护：一旦异常直接回到低油门，避免飞控发散。
    for(int i = 0; i < 4; ++i){
        if(!std::isfinite(pwm[i])){
            pwm = Eigen::Vector4f::Constant(pwm_min);
            break;
        }
    }

    float min_pwm = pwm.minCoeff();
    if(min_pwm < pwm_min){
        float offset = pwm_min - min_pwm;
        pwm = pwm.array() + offset;
    }

    // 再做上限归一化：避免出现 >1 的情况（按比例缩放）。
    float max_pwm = pwm.maxCoeff();
    if(max_pwm > 1.0f){
        pwm = pwm.array() / max_pwm;
    }

    // 最终再保险一下下限（理论上不会再触发到 < pwm_min，但用于数值误差）。
    for(int i = 0; i < 4; ++i){
        pwm[i] = std::max<float>(pwm[i], pwm_min);
        pwm[i] = std::min<float>(pwm[i], 1.0f);
    }

    return pwm;
}