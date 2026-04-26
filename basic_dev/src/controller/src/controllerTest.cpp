#include <algorithm>
#include <cmath>

#include <airsim_ros/RotorPWM.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class DroneController
{
private:
    struct PIDParams
    {
        double kp;
        double ki;
        double kd;
        double integral;
        double prev_error;

        PIDParams() : kp(0.0), ki(0.0), kd(0.0), integral(0.0), prev_error(0.0) {}
    };

    static constexpr double BASE_PWM = 0.6;
    static constexpr double MAX_PWM = 0.8;
    static constexpr double MIN_PWM = 0.4;
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double MAX_TILT_CMD = 0.35;
    static constexpr double YAW_RATE_TO_TARGET_SCALE = 0.1;

    ros::NodeHandle nh_;
    ros::Publisher pwm_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber cmd_vel_sub_;

    PIDParams roll_pid_;
    PIDParams pitch_pid_;
    PIDParams yaw_pid_;

    double current_roll_;
    double current_pitch_;
    double current_yaw_;

    double target_roll_;
    double target_pitch_;
    double target_yaw_;

    ros::Time last_time_;
    bool imu_initialized_;

    static double wrapAngle(double angle)
    {
        while (angle > PI)
        {
            angle -= 2.0 * PI;
        }
        while (angle < -PI)
        {
            angle += 2.0 * PI;
        }
        return angle;
    }

    double updatePID(PIDParams &pid, double error, double dt)
    {
        if (dt <= 1e-6)
        {
            return pid.kp * error;
        }

        pid.integral += error * dt;
        const double derivative = (error - pid.prev_error) / dt;
        pid.prev_error = error;
        return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    }

public:
    DroneController()
        : current_roll_(0.0),
          current_pitch_(0.0),
          current_yaw_(0.0),
          target_roll_(0.0),
          target_pitch_(0.0),
          target_yaw_(0.0),
          imu_initialized_(false)
    {
        pwm_pub_ =
            nh_.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 10);
        imu_sub_ = nh_.subscribe("/airsim_node/drone_1/imu/imu", 50, &DroneController::imuCallback, this);
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 20, &DroneController::cmdVelCallback, this);

        initPIDParams();
        ROS_INFO("Drone PWM controller initialized");
    }

    void initPIDParams()
    {
        roll_pid_.kp = 0.10;
        roll_pid_.ki = 0.01;
        roll_pid_.kd = 0.05;

        pitch_pid_.kp = 0.10;
        pitch_pid_.ki = 0.01;
        pitch_pid_.kd = 0.05;

        yaw_pid_.kp = 0.15;
        yaw_pid_.ki = 0.00;
        yaw_pid_.kd = 0.10;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 mat(quat);
        mat.getRPY(current_roll_, current_pitch_, current_yaw_);

        if (!imu_initialized_)
        {
            target_roll_ = current_roll_;
            target_pitch_ = current_pitch_;
            target_yaw_ = current_yaw_;
            last_time_ = ros::Time::now();
            imu_initialized_ = true;
            ROS_INFO("IMU initialized, entering PWM control loop");
            return;
        }

        const ros::Time current_time = ros::Time::now();
        const double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        const double roll_error = target_roll_ - current_roll_;
        const double pitch_error = target_pitch_ - current_pitch_;
        const double yaw_error = wrapAngle(target_yaw_ - current_yaw_);

        const double roll_output = updatePID(roll_pid_, roll_error, dt);
        const double pitch_output = updatePID(pitch_pid_, pitch_error, dt);
        const double yaw_output = updatePID(yaw_pid_, yaw_error, dt);

        const double pwm0 = BASE_PWM + pitch_output - roll_output - yaw_output;
        const double pwm1 = BASE_PWM - pitch_output + roll_output - yaw_output;
        const double pwm2 = BASE_PWM - pitch_output - roll_output + yaw_output;
        const double pwm3 = BASE_PWM + pitch_output + roll_output + yaw_output;

        airsim_ros::RotorPWM pwm_msg;
        pwm_msg.header.stamp = current_time;
        pwm_msg.rotorPWM0 = std::max(MIN_PWM, std::min(MAX_PWM, pwm0));
        pwm_msg.rotorPWM1 = std::max(MIN_PWM, std::min(MAX_PWM, pwm1));
        pwm_msg.rotorPWM2 = std::max(MIN_PWM, std::min(MAX_PWM, pwm2));
        pwm_msg.rotorPWM3 = std::max(MIN_PWM, std::min(MAX_PWM, pwm3));
        pwm_pub_.publish(pwm_msg);
    }

    void cmdVelCallback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        if (!imu_initialized_)
        {
            return;
        }

        target_pitch_ = std::max(-MAX_TILT_CMD, std::min(MAX_TILT_CMD, -msg->x * 0.2));
        target_roll_ = std::max(-MAX_TILT_CMD, std::min(MAX_TILT_CMD, msg->y * 0.2));
        target_yaw_ = wrapAngle(target_yaw_ + msg->z * YAW_RATE_TO_TARGET_SCALE);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_pwm_controller");
    DroneController controller;
    ros::spin();
    return 0;
}