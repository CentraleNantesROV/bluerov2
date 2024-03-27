# BlueROV2 ROS 2

This repository contains the robot description and necessary launch files to describe and simulate the BlueROV2 (unmanned underwater vehicle) with [Gazebo](https://gazebosim.org/home) and its [hydrodynamics plugins](https://gazebosim.org/api/gazebo/6.1/underwater_vehicles.html) under ROS 2.


## Requirements

### For the description

- [Xacro ](https://github.com/ros/xacro/tree/ros2), installable through `apt install ros-${ROS_DISTRO}-xacro`
- [simple_launch](https://github.com/oKermorgant/simple_launch), installable through `apt install ros-${ROS_DISTRO}-simple-launch`

### Gazebo

- ROS 2 with Gazebo Fortress or newer with `ros_gz_bridge`
    - Look out for your [ROS 2 / Gazebo versions combination](https://gazebosim.org/docs/garden/ros_installation)
- [pose_to_tf](https://github.com/oKermorgant/pose_to_tf), to get the ground truth from Gazebo if needed.

### For the control part

- [slider_publisher](https://github.com/oKermorgant/slider_publisher), installable through `apt install ros-${ROS_DISTRO}-slider-publisher`
- [auv_control](https://github.com/CentraleNantesROV/auv_control) for basic control laws, from source


## Installation 

Clone the package and its dependencies (if from source) in your ROS 2 workspace `src` and compile with `colcon`

## Running 

To run a demonstration with the vehicle, you can run a Gazebo scenario, such as an empty world with buoyancy and sensors setup:

```bash
ros2 launch bluerov2_description world_launch.py
```

and then spawn the robot with a GUI to control the thrusters:

```bash
ros2 launch bluerov2_description upload_bluerov2_launch.py sliders:=true
```

## Input / output

Gazebo will:

- Subscribe to `/bluerov2/cmd_thruster[1..6]` and expect  `std_msgs/Float64` messages, being the thrust in Newton
- Publish sensor data to various topics (image, mpu+lsm for IMU, cloud for the sonar, odom)
- Publish the ground truth on `/bluerov2/pose_gt`. This pose is forwarded to `/tf` if `pose_to_tf` is used.


## High-level control

Basic control is available in the [auv_control](https://github.com/CentraleNantesROV/auv_control) package

In this case spawn the robot without manual sliders and run e.g. a cascaded PID controller:

```bash
ros2 launch bluerov2_description upload_bluerov2_launch.py
ros2 launch bluerov2_control cascaded_pids_launch.py sliders:=true
```


## License

BlueROV2 package is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.
