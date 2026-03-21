# Folder Role: Advanced Odometry & Mapping
This folder is dedicated to higher-level navigation tasks, including map management and potentially visual/LiDAR-based positioning.

## Files and Purposes
- **`src/odometry.cpp`**: Main node for managing odometry information.
- **`src/keyframe.cpp`**: Logic for managing "keyframes," which are snapshots of the drone's state used to build and optimize a map.
- **`src/mapmanager.cpp`**: Handles the storage and retrieval of mapped environment data.
- **`launch/odometry.launch`**: Launch file for the navigation system.
- **`launch/config.rviz`**: Configuration file for the RViz visualizer to monitor the drone's estimated path and map.
