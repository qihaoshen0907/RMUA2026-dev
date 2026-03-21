# Folder Role: Basic Development & Orchestration
This is the primary workspace for high-level mission logic and state machine implementation for the RoboMaster AI Challenge.

## Files and Purposes
- **`src/basic_dev.cpp`**: The main ROS node. It subscribes to sensor data (GPS, IMU, Camera) and coordinates the drone's behavior by calling services or publishing commands.
- **`include/basic_dev.hpp`**: Header file defining the `BasicDev` class and its callback structures.
- **`CMakeLists.txt` & `package.xml`**: ROS build configuration for the core logic.
