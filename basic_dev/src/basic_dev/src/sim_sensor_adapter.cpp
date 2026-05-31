/**
 * sim_sensor_adapter.cpp
 * 
 * NED → ENU sensor adapter for AirSim → FAST-LIVO2 pipeline.
 * 
 * Subscribes to AirSim topics (NED frame) and republishes them
 * in ENU frame so that FAST-LIVO2 (and other ROS SLAM systems)
 * can consume them directly.
 * 
 * Transformations applied:
 *   - LiDAR points:     [x,y,z]_enu = [y, x, -z]_ned
 *   - IMU accel/gyro:   same vector rotation as LiDAR
 *   - IMU orientation:  quaternion pre-multiplied by NED→ENU rotation
 *   - Camera image:     header.frame_id updated to ENU convention
 * 
 * The IMU timestamp is used as the master clock reference; if available,
 * LiDAR and camera messages are restamped with the latest IMU time to
 * guarantee tight temporal alignment for the downstream SLAM node.
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <cmath>

class SimSensorAdapter
{
public:
    SimSensorAdapter(ros::NodeHandle& nh)
        : m_lidar_count(0), m_imu_count(0), m_camera_count(0)
    {
        // ── Subscribers ──
        m_lidar_sub = nh.subscribe("/airsim_node/drone_1/lidar", 10,
                                   &SimSensorAdapter::lidarCallback, this);
        m_imu_sub   = nh.subscribe("/airsim_node/drone_1/imu/imu", 200,
                                   &SimSensorAdapter::imuCallback, this);
        m_camera_sub = nh.subscribe("/airsim_node/drone_1/front_left/Scene", 10,
                                    &SimSensorAdapter::cameraCallback, this);

        // ── Publishers ──
        m_lidar_pub  = nh.advertise<sensor_msgs::PointCloud2>("/sim/lidar", 10);
        m_imu_pub    = nh.advertise<sensor_msgs::Imu>("/sim/imu", 200);
        m_camera_pub = nh.advertise<sensor_msgs::Image>("/sim/camera", 10);

        // ── NED → ENU static rotation ──
        // NED: X=North, Y=East, Z=Down
        // ENU: X=East, Y=North, Z=Up
        m_R_ned_to_enu << 0.0, 1.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, 0.0, -1.0;

        // Quaternion representation of the above rotation
        // (180° around axis [1/√2, 1/√2, 0])
        m_q_ned_to_enu = Eigen::Quaterniond(0.0,
                                            1.0 / std::sqrt(2.0),
                                            1.0 / std::sqrt(2.0),
                                            0.0);

        ROS_INFO("sim_sensor_adapter: initialised.");
        ROS_INFO("  LiDAR in : /airsim_node/drone_1/lidar");
        ROS_INFO("  IMU in   : /airsim_node/drone_1/imu/imu");
        ROS_INFO("  Camera in: /airsim_node/drone_1/front_left/Scene");
        ROS_INFO("  LiDAR out : /sim/lidar");
        ROS_INFO("  IMU out   : /sim/imu");
        ROS_INFO("  Camera out: /sim/camera");
    }

    // -----------------------------------------------------------------
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        sensor_msgs::PointCloud2 out = *msg;
        out.header.frame_id = "lidar_enu";

        // Use IMU timestamp as master clock when available
        if (!m_latest_imu_time.isZero())
            out.header.stamp = m_latest_imu_time;

        sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");

        const std::size_t num_points = out.width * out.height;
        for (std::size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            Eigen::Vector3f p_ned(*iter_x, *iter_y, *iter_z);
            Eigen::Vector3f p_enu = m_R_ned_to_enu.cast<float>() * p_ned;
            *iter_x = p_enu.x();
            *iter_y = p_enu.y();
            *iter_z = p_enu.z();
        }

        m_lidar_pub.publish(out);

        if (++m_lidar_count == 1)
            ROS_INFO("sim_sensor_adapter: first LiDAR message published (%zu pts).", num_points);
    }

    // -----------------------------------------------------------------
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        m_latest_imu_time = msg->header.stamp;

        sensor_msgs::Imu out = *msg;
        out.header.frame_id = "imu_enu";

        // --- linear acceleration ---
        Eigen::Vector3d acc_ned(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
        Eigen::Vector3d acc_enu = m_R_ned_to_enu * acc_ned;
        out.linear_acceleration.x = acc_enu.x();
        out.linear_acceleration.y = acc_enu.y();
        out.linear_acceleration.z = acc_enu.z();

        // --- angular velocity ---
        Eigen::Vector3d gyro_ned(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);
        Eigen::Vector3d gyro_enu = m_R_ned_to_enu * gyro_ned;
        out.angular_velocity.x = gyro_enu.x();
        out.angular_velocity.y = gyro_enu.y();
        out.angular_velocity.z = gyro_enu.z();

        // --- orientation (if provided) ---
        // sensor_msgs convention: covariance[0] == -1 means "unknown orientation"
        if (msg->orientation_covariance[0] != -1.0)
        {
            Eigen::Quaterniond q_ned(msg->orientation.w,
                                     msg->orientation.x,
                                     msg->orientation.y,
                                     msg->orientation.z);
            Eigen::Quaterniond q_enu = m_q_ned_to_enu * q_ned;
            q_enu.normalize();
            out.orientation.w = q_enu.w();
            out.orientation.x = q_enu.x();
            out.orientation.y = q_enu.y();
            out.orientation.z = q_enu.z();
        }

        m_imu_pub.publish(out);

        if (++m_imu_count == 1)
            ROS_INFO("sim_sensor_adapter: first IMU message published.");
    }

    // -----------------------------------------------------------------
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        sensor_msgs::Image out = *msg;
        out.header.frame_id = "front_left_camera_enu";

        // Use IMU timestamp as master clock when available
        if (!m_latest_imu_time.isZero())
            out.header.stamp = m_latest_imu_time;

        m_camera_pub.publish(out);

        if (++m_camera_count == 1)
            ROS_INFO("sim_sensor_adapter: first camera message published.");
    }

private:
    ros::Subscriber m_lidar_sub;
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_camera_sub;

    ros::Publisher m_lidar_pub;
    ros::Publisher m_imu_pub;
    ros::Publisher m_camera_pub;

    Eigen::Matrix3d m_R_ned_to_enu;
    Eigen::Quaterniond m_q_ned_to_enu;
    ros::Time m_latest_imu_time;

    std::size_t m_lidar_count;
    std::size_t m_imu_count;
    std::size_t m_camera_count;
};

// ===================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_sensor_adapter");
    ros::NodeHandle nh("~");
    SimSensorAdapter adapter(nh);
    ros::spin();
    return 0;
}
