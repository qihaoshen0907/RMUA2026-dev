#pragma once
#include "Eigen/Dense"
class UAVLinearController
{
public:
    //X=[x, y, z, vx, vy, vz, phi, theta, psi, wphi, wtheta, wpsi];
    Eigen::Vector4f execute(Eigen::VectorXf& X_des, Eigen::VectorXf& X_real);
    void resetErrorTerms(const Eigen::VectorXf& X_des, const Eigen::VectorXf& X_real);

private:
    int m_cnt = 0;

    // physical parameters
    float m_mass = 0.9;
    float m_gravity = 9.8;
    float m_armlength = 0.18;
    float m_Ixx = 0.0046890742f;
    float m_Iyy = 0.0069312f;
    float m_Izz = 0.010421166f;
    float m_Ct = 0.00036771704516278653f;
    float m_Cq = 4.888486266072161e-06f;
    float m_Fmax = 4.179446268f * 3.0f;

    // x position outer loop
    float m_Kdx = 0.10f;
    float m_Kpx = 0.80f;
    float m_last_e_x = 0.0f;

    // y position outer loop
    float m_Kdy = 0.02f;
    float m_Kpy = 0.20f;
    float m_last_e_y = 0.0f;

    // z altitude loop
    float m_Kdz = 6.0f;
    float m_Kpz = 1.2f;
    float m_last_e_z = 0.0f;

    // roll attitude loop
    float m_Kdphi = 28.0f;
    float m_Kpphi = 5.5f;
    float m_last_e_phi = 0.0f;

    // pitch attitude loop
    float m_Kdtheta = 28.0f;
    float m_Kptheta = 5.5f;
    float m_last_e_theta = 0.0f;

    // yaw loop
    float m_Kdpsi = 10.0f;
    float m_Kppsi = 1.5f;
    float m_last_e_psi = 0.0f;

    // x velocity inner loop
    float m_Kdvx = 0.30f;
    float m_Kpvx = 0.65f;
    float m_last_e_vx = 0.0f;

    // y velocity inner loop
    float m_Kdvy = 0.10f;
    float m_Kpvy = 0.20f;
    float m_last_e_vy = 0.0f;

    // limits
    float m_max_acc = 5.5f;
    float m_max_v   = 10.5f;
    float m_max_ang = 0.12f;

    float m_last_thetad = 0.0f;
};
// private:
//     int m_cnt = 0;
//     float m_mass = 0.9;
//     float m_gravity = 9.8;
//     float m_armlength = 0.18;
//     float m_Ixx = 0.0046890742;//kg.m^2
//     float m_Iyy = 0.0069312;
//     float m_Izz = 0.010421166;
//     float m_Ct = 0.00036771704516278653;
//     float m_Cq = 4.888486266072161e-06;
//     float m_Fmax = 4.179446268*3;
//     float m_Kdx = 0.1;
//     float m_Kpx = 1.0;
//     float m_last_e_x = 0;
//     float m_Kdy = 0.1;
//     float m_Kpy = 1.0;
//     float m_last_e_y = 0;
//     float m_Kdz = 25.0;
//     float m_Kpz = 2.0;
//     float m_last_e_z = 0;
//     float m_Kdphi = 940; 
//     float m_Kpphi = 17;
//     float m_last_e_phi = 0;
//     float m_Kdtheta = 750;
//     float m_Kptheta = 20;
//     float m_last_e_theta = 0;
//     float m_Kdpsi = 250;
//     float m_Kppsi = 5;
//     float m_last_e_psi = 0;
//     float m_Kdvx = 1.0;
//     float m_Kpvx = 0.5;
//     float m_last_e_vx = 0;
//     float m_Kdvy = 1.0;
//     float m_Kpvy = 0.5;
//     float m_last_e_vy = 0;
//     float m_max_acc = 20;
//     float m_max_v = 20;
//     float m_max_ang = 0.5;
//     float m_last_thetad = 0;
// };