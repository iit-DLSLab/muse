# Base ROS image
FROM ros:noetic-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install dependencies and Pinocchio
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ros-noetic-dynamic-reconfigure \
    libeigen3-dev \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-rviz \
    python3-catkin-tools \
    python3-catkin-pkg \
    python3-rospkg \
    cmake \
    python3-rosdep \
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglu1-mesa \
    freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*

ENV LIBGL_ALWAYS_SOFTWARE=1

# create and initialize a catkin workspace
WORKDIR /home/dev_ws
RUN mkdir -p catkin_ws/src

# # clone kiss-icp ROS wrappers (v0.3.0)
WORKDIR /home/dev_ws/catkin_ws/src
RUN git clone --branch v0.3.0 https://github.com/PRBonn/kiss-icp.git

# install any remaining ROS dependencies
WORKDIR /home/dev_ws/catkin_ws
RUN \
  rosdep init || true && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y

# build the workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build"

# automatically source both ROS and your workspace on container start
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /home/dev_ws/catkin_ws/devel/setup.bash" >> ~/.bashrc

# create a safe runtime dir for Qt
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# default to bash
CMD ["bash"]

# Gazebo
RUN apt-get update && apt-get install -y \
    gazebo11 libgazebo11-dev \
    ros-noetic-gazebo-ros-* \
    && rm -rf /var/lib/apt/lists/*


## ROS Control
RUN apt-get update && apt-get install -y \
  ros-noetic-controller-interface \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-joint-state-controller \
  ros-noetic-effort-controllers \
  ros-noetic-joint-trajectory-controller \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  ros-noetic-joint-state-publisher