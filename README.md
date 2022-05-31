# BlueROV2 ROS 2 Simulation

This repository contains the robot description and necessary launch files to simulate the BlueROV2 (unmanned underwater vehicle) with [ignition-gazebo](https://ignitionrobotics.org/libs/gazebo) and its [hydrodynamics plugins](https://ignitionrobotics.org/api/gazebo/5.0/underwater_vehicles.html).

It is a ROS 2 fork of the work in development at [Ingeniarius, Lda.](http://ingeniarius.pt/) and [Institute of Systems and Robotics University of Coimbra](https://www.isr.uc.pt/) within the scope of MS thesis "Localization of an unmanned underwater vehicle using multiple water surface robots, multilateration, and sensor data fusion".


## Requirements

- ROS 2 Foxy or newer with `ign-gazebo` and `ign-bridge`
- [simple_launch](https://github.com/oKermorgant/simple_launch), installable through `apt install ros-foxy-simple-launch`

## Installation 

Clone the package in your ROS 2 workspace `src` and compile with `colcon`

## Running 

To run a demonstration with the vehicle, you can run a UUV simulator Gazebo scenario, such as

```bash
ign gazebo graded_buoyancy.sdf
```

and then

```bash
ros2 launch bluerov2_description upload_bluerov2_launch.py
```

## License

BlueROV2 package is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.
