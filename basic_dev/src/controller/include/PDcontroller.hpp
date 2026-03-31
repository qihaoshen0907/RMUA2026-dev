#pragma once
#include "Eigen/Dense"
class UAVLinearController
{
public:
    //X=[x, y, z, vx, vy, vz, phi, theta, psi, wphi, wtheta, wpsi];
    Eigen::Vector4f execute(Eigen::VectorXf& X_des, Eigen::VectorXf& X_real);

private:
    int m_cnt = 0;
    float m_mass = 0.9;
    float m_gravity = 9.8;
    float m_armlength = 0.18;
    float m_Ixx = 0.0046890742;//kg.m^2
    float m_Iyy = 0.0069312;
    float m_Izz = 0.010421166;
    float m_Ct = 0.00036771704516278653;
    float m_Cq = 4.888486266072161e-06;
    float m_Fmax = 4.179446268*3;
    float m_Kdx = 0.1;
    float m_Kpx = 1.0;
    float m_last_e_x = 0;
    float m_Kdy = 0.1;
    float m_Kpy = 1.0;
    float m_last_e_y = 0;
    float m_Kdz = 25.0;
    float m_Kpz = 4.0;
    float m_Kvz_damp = 3.0;
    float m_last_e_z = 0;
    // 原参数在部分机器/帧率/延迟条件下容易导致姿态“跳舞”（高频振荡）
    // 先把 D 项降下来，稳定后再逐步调高响应
    float m_Kdphi = 8;
    float m_Kpphi = 3;
    float m_last_e_phi = 0;
    float m_Kdtheta = 8;
    float m_Kptheta = 3;
    float m_last_e_theta = 0;
    float m_Kdpsi = 4;
    float m_Kppsi = 0.8;
    float m_last_e_psi = 0;
    float m_Kdvx = 1.0;
    float m_Kpvx = 0.5;
    float m_last_e_vx = 0;
    float m_Kdvy = 1.0;
    float m_Kpvy = 0.5;
    float m_last_e_vy = 0;
    float m_max_acc = 3;
    float m_max_v = 2.5;
    float m_max_ang = 0.35;
    float m_last_thetad = 0;
};