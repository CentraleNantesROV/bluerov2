# Lightweight sensor snippets for xacro files in Plankton

These files are adapted from (uuv_sensor_ros_plugins)[https://github.com/Liquid-ai/Plankton/tree/master/uuv_sensor_plugins/uuv_sensor_ros_plugins].

The main difference is that the links attached to the sensors are fixed and all offsets are part of the Gazebo sensor information.

This allows a lighter simulation while sensor frames are still available on `\tf`.

Also, it removes the dependency on the whole Plankton stack when using this package onboard.
