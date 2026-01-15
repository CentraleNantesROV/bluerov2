# ROS 2 Humble + Ignition Gazebo Fortress

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/opt/ros_ws
ENV GIT_TERMINAL_PROMPT=0


# Base tools
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Ignition Gazebo Fortress
RUN apt-get update && apt-get install -y \
    ignition-fortress \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-ign-image \
    ros-humble-xacro \
    ros-humble-simple-launch \
    ros-humble-slider-publisher \
    && rm -rf /var/lib/apt/lists/*


# rosdep
RUN rosdep init || true
RUN rosdep update


# Workspace
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}


# Copy local packages
COPY bluerov2_description ${ROS_WS}/src/bluerov2_description
COPY bluerov2_control     ${ROS_WS}/src/bluerov2_control


# External dependencies
RUN cd src && \
    git clone https://github.com/CentraleNantesROV/thruster_manager.git && \
    git clone https://github.com/CentraleNantesROV/auv_control.git && \
    git clone https://github.com/oKermorgant/pose_to_tf.git


# Install dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y


# Build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
      --symlink-install \
      --parallel-workers $(nproc)


# DDS / Networking
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
