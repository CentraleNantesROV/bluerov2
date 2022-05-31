cmake_minimum_required(VERSION 3.5)
project(bluerov2_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/rviz_bridge.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
