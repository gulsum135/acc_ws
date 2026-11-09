# acc_ws

ROS 2 workspace for Adaptive Cruise Control (ACC) and Automatic Emergency Braking (AEB) experiments.

## Packages

- `acc_sim`: original lightweight Python simulation.
- `acc_gazebo_sim`: Gazebo Sim + ROS 2 integration baseline with a differential-drive ego vehicle, GPU LiDAR, odometry bridge, and ACC/AEB controller.

## Gazebo simulation

The Gazebo integration currently targets **ROS 2 Jazzy + Gazebo Harmonic**.

See [Gazebo setup and run guide](docs/GAZEBO_SETUP.md) for dependencies, build instructions, launch command, topics, parameters, and current limitations.

Quick start after installing the dependencies:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/acc_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch acc_gazebo_sim gazebo.launch.py
```

This repository is an educational simulation project, not a production or safety-certified driver assistance system.
