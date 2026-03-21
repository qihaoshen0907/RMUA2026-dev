# Folder Role: AirSim-ROS Bridge
This folder acts as the interface between the AirSim simulator and the ROS environment. It defines the communication protocols used to interact with the drone.

## Files and Purposes
- **`msg/*.msg`**: Custom ROS message definitions specific to AirSim (e.g., `VelCmd` for velocity control, `RotorPWM` for motor signals, `CarState`).
- **`srv/*.srv`**: Custom ROS service definitions for controlling simulator state (e.g., `Takeoff`, `Land`, `Reset`).
- **`CMakeLists.txt` & `package.xml`**: Standard ROS build and dependency configuration.
