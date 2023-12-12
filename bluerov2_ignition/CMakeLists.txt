cmake_minimum_required(VERSION 3.5)
project(bluerov2_ignition)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
