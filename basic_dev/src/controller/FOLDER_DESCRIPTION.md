# Folder Role: Flight Control System
This folder contains the algorithms responsible for translating desired positions or velocities into low-level motor commands (PWM).

## Files and Purposes
- **`src/PDcontroller.cpp`**: Implements a cascaded PD (Proportional-Derivative) controller for position, velocity, and attitude control.
- **`src/controllerTest.cpp`**: A utility node used to verify and tune the controller's performance.
- **`include/PDcontroller.hpp`**: Defines the controller gains and the execution logic.
- **`launch/controller_test.launch`**: ROS launch file to start the controller and testing utilities.
- **`src/Splines.txt`**: Likely contains trajectory or waypoint data for testing smooth flight paths.
