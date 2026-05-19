# Folder Role: Flight Control System
This folder contains the algorithms responsible for translating desired positions or velocities into low-level motor commands (PWM).

## Files and Purposes
- **`src/mpc_controller.cpp`**: Implements the linearized quadrotor model, MPC solver loop, input/state constraints, and rotor PWM mixer.
- **`include/mpc_controller.hpp`**: Declares MPC model parameters, cost weights, constraints, and solver interfaces.
- **`src/controllerTest.cpp`**: ROS node glue code. Subscribes to odometry/goal topics, keeps frame conversion logic, builds the 12D state/reference, and publishes rotor PWM.
- **`src/PDcontroller.cpp`**: Legacy cascaded PD controller retained for reference.
- **`include/PDcontroller.hpp`**: Legacy PD controller interface.
- **`launch/controller_test.launch`**: ROS launch file to start the controller and testing utilities.
- **`src/Splines.txt`**: Likely contains trajectory or waypoint data for testing smooth flight paths.
