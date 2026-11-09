# Gazebo ACC/AEB simulation

This package adds a physical differential-drive vehicle to Gazebo Sim and connects it to ROS 2. The existing `acc_sim` package is intentionally left unchanged.

## Supported baseline

The launch setup targets **ROS 2 Jazzy + Gazebo Harmonic** with the `ros_gz` packages installed. Other ROS/Gazebo pairings may need package or plugin adjustments.

## Install dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

Source ROS 2 and build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/acc_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch acc_gazebo_sim gazebo.launch.py
```

The world starts with the ego vehicle at the origin and a static obstacle 18 m ahead. The front GPU LiDAR publishes `/scan`; Gazebo odometry is bridged to `/odom`; the ROS controller publishes velocity commands to `/cmd_vel`.

Useful inspection commands:

```bash
ros2 topic list
ros2 topic echo /sim/state
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

## Controller behavior

- Uses the minimum valid LiDAR range in a configurable forward angular sector.
- Uses odometry for measured forward speed.
- Reduces desired speed using a comfort-braking stopping-distance curve.
- Applies a stronger deceleration request inside the critical range.
- Commands zero speed if LiDAR data becomes stale (fail-safe behavior).

Parameters are declared by `gazebo_acc_controller`, for example:

```bash
ros2 param set /gazebo_acc_controller target_speed_mps 3.0
```

## Important limitations

This is an integration baseline, not a validated autonomous-driving or safety system. The obstacle is static, the vehicle is a simple differential-drive model, and the controller uses a single forward LiDAR sector. Before treating results as meaningful performance evidence, verify the SDF and plugin loading on the target machine, check wheel contact and odometry, then add automated scenario tests (moving lead vehicle, cut-in, sensor dropout, and braking-distance sweeps).
