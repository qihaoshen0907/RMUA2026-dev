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

For Zsh
```zsh
source /opt/ros/noetic/setup.zsh
```

## 3. Compilation

**Note: Everything below this line only works with the `keyboard` branch. Adjust the code accordingly.**

Go to the work directory first and delete old compiled files
```bash
cd ~/RMUA2026-dev/basic_dev/
rm -rf source devel
```

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_ros;basic_dev"
```

## 4. Source the compiled file

Bash
```bash
source devel/setup.bash
```

Zsh
```zsh
source devel/setup.zsh
```

## 5. Run with rosrun

```bash
rosrun basic_dev keyboard_teleop.py
```


# Record Position
## 1. Enter directory *basic_dev* like before
## 2. Source ROS environment
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

## 3. Subscribe Topic
```bash
rostopic echo /airsim_node/drone_1/debug/pose_gt
```

# Save historical positions into a .log file
```bash
rostopic echo /airsim_node/drone_1/debug/pose_gt > drone_pose.log
```

