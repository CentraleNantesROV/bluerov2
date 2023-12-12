# BlueROV2 ROS 2

This repository contains the robot description and necessary launch files to describe and simulate the BlueROV2 (unmanned underwater vehicle) with [Gazebo](https://gazebosim.org/home) and its [hydrodynamics plugins](https://gazebosim.org/api/gazebo/6.1/underwater_vehicles.html) under ROS 2.


## Requirements

- ROS 2 Garden or newer with `ros_gz_bridge` (you may have to [recompile it from sources](https://github.com/gazebosim/ros_gz))
- [simple_launch](https://github.com/oKermorgant/simple_launch), installable through `apt install ros-${ROS_DISTRO}-simple-launch`
- [slider_publisher](https://github.com/oKermorgant/slider_publisher), installable through `apt install ros-${ROS_DISTRO}-slider-publisher`
- [pose_to_tf](https://github.com/oKermorgant/pose_to_tf), to get the ground truth from Gazebo.

### Foxy / Ignition

A legacy version using Ignition plugins is available in `bluerov2_ignition`.

## Installation 

Clone the package in your ROS 2 workspace `src` and compile with `colcon`

## Running 

To run a demonstration with the vehicle, you can run a Gazebo scenario, such as an empty world with buoyancy and sensors setup:

```bash
ros2 launch bluerov2_description world_launch.py
```

and then spawn the robot with a GUI to control the thrusters:

```bash
ros2 launch bluerov2_description upload_bluerov2_launch.py sliders:=true
```

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
