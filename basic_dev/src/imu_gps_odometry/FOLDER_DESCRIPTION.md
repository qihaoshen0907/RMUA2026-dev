# Folder Role: IMU-GPS State Estimation
This folder handles the fusion of IMU and GPS data to provide a reliable and smooth estimate of the drone's state (position, velocity, orientation).

## Files and Purposes
- **`src/eskf.cpp`**: Implementation of the **Error-State Kalman Filter (ESKF)**, which is the mathematical core of the state estimation.
- **`src/imu_gps_odometry.cpp`**: The ROS wrapper node that feeds raw sensor data into the ESKF and publishes the filtered result.
- **`include/eskf.hpp`**: Header defining the ESKF mathematical model and noise parameters.
- **`CMakeLists.txt` & `package.xml`**: Build configuration for the estimation system.
