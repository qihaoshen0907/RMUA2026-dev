# Compiling the basic_dev w/o Docker

Basically, we do everything the [Dockerfile](./basic_dev/Dockerfile) does one by one. 

## 1. Install necessary packages

```bash
sudo apt update && sudo apt install -y python3-catkin-tools ros-noetic-geographic-msgs ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport net-tools
```

## 2. Make sure ROS is properly sourced

For Bash (the default Ubuntu shell interpreter)
```bash
source /opt/ros/noetic/setup.bash
```

## 3. Compilation

**Note: Everything below this line only works with the `keyboard` branch. Adjust the code accordingly.**

Go to the work directory first and delete old compiled files 
(You must do it if they're **not** compiled on your PC). 
```bash
cd ~/RMUA2026-dev/basic_dev/
rm -rf source devel
```

```bash
# Example 1: compile the manual keyboard control node
catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_ros;basic_dev"

# Example 2: compile the PD controlled spline following nodes
catkin_make --only-pkg-with-deps airsim_ros imu_gps_odometry controller_test
```

## 4. Source the compiled file

You can also add the `source devel/setup.bash` and `source /opt/ros/noetic/setup.bash` to your `~/.bashrc`. 
By doing so, your bash will source them automatically when you open a new terminal window. 

Bash
```bash
source devel/setup.bash
```

## 5. Run the nodes

```bash
# First thing first
# Open a separate terminal window and cd to the simulator folder
cd ~/path_to_your_simulator_files/simulator_12.0.0.4

# Run the simulator
./run_simulator.sh 123

# Example 1: run the manual keyboard control node
rosrun basic_dev keyboard_teleop.py

# Example 2: run the PD controlled spline following nodes
# (sequence doesn't matter but the two nodes needs to be running in two separate terminals)
# Terminal window 1:
rosrun imu_gps_odometry imu_gps_odometry
# Terminal window 2:
rosrun controller_test controller_test
```

# How to monitor realtime coordinate/pose of the drone
## 1. Enter directory *basic_dev* like before
## 2. Source ROS environment
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 3. Subscribe Topic
```bash
rostopic echo /airsim_node/drone_1/debug/pose_gt
```
