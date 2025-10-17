# Base ROS 2 Galactic mínima
FROM ros:galactic-ros-core

# Herramientas básicas
RUN apt-get update && apt-get install -y \
    git nano curl gnupg lsb-release && rm -rf /var/lib/apt/lists/*

# Repos de OSRF (Gazebo 11)
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    | gpg --dearmor > /etc/apt/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Gazebo 11 + integración ROS 2 Galactic
RUN apt-get update && apt-get install -y \
    gazebo11 libgazebo11-dev \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-gazebo-ros2-control \
    ros-galactic-ros2-control \
    ros-galactic-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

## ROS Control
RUN apt-get update && apt-get install -y \
  ros-galactic-controller-interface \
  ros-galactic-ros-control \
  ros-galactic-ros-controllers \
  ros-galactic-gazebo-ros-pkgs \
  ros-galactic-gazebo-ros-control \
  ros-galactic-joint-state-controller \
  ros-galactic-effort-controllers \
  ros-galactic-joint-trajectory-controller \
  ros-galactic-robot-state-publisher
