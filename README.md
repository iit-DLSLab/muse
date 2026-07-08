<h1 align="center"> MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots </h1>
<h3 align="center">Ylenia Nisticò, João Carlos Virgolino Soares, Lorenzo Amatucci, Geoff Fink and Claudio Semini</h3>

<h4 align="center">This paper has been accepted to IEEE Robotics and Automation Letters, and it is available at https://arxiv.org/abs/2503.12101 </h4>

# :computer: Overview

MUSE provides a ROS1 state-estimator node for quadruped robots. The node loads estimator modules with `pluginlib`, reads proprioceptive robot data, and publishes attitude, contact, leg odometry, fused odometry, and TF outputs.

The current ROS1 implementation is proprioceptive. The necessary online inputs are:

- IMU measurements
- actuator readings

It also requires a robot URDF and matching foot/joint frame names in the configuration.

Default configuration files are provided for ANYmal-style naming, with additional URDFs for `aliengo`, `anymal`, and `go1` in `muse_ws/src/state_estimator/urdfs`.

## :t-rex: Prerequisites

- Conda or Mamba
- Dependencies from the provided `environment.yml`

The conda environment is named `muse` and provides the ROS1 Noetic stack, `catkin_tools`, Eigen, Pinocchio, RViz, PlotJuggler, and the ROS packages used by the workspace.

Create the environment with:

```sh
cd muse
mamba env create -f environment.yml
```

If the environment already exists, update it with:

```sh
mamba env update -n muse -f environment.yml --prune
```

## :hammer_and_wrench: Building

Build the ROS1 workspace with `catkin build`:

```sh
cd muse/muse_ws
conda activate muse
catkin init
catkin config --source-space src --build-space build --devel-space devel --install-space installcatkin build
source install/setup.bash
```

## :rocket: Running

Launch the estimator:

```sh
roslaunch state_estimator state_estimator.launch
```

RViz is disabled by default:

```sh
roslaunch state_estimator state_estimator.launch rviz:=false
```

For rosbag playback, use simulated time and play the bag with `/clock`:

```sh
roslaunch state_estimator state_estimator.launch use_sim_time:=true
rosbag play --clock your_rosbag.bag
```

## :electric_plug: ROS Interfaces

Default input topics:

- `/anymal/imu` (`sensor_msgs/Imu`)
- `/anymal/state_estimator/anymal_state` (the definition of the anymal msgs is taken from the [holistic_fusion](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros/graph_msf_anymal_msgs/msg) repo from ETH)

Default estimator outputs are published in the private node namespace:

- `/state_estimator/attitude` (`state_estimator_msgs/attitude`)
- `/state_estimator/contact_detection` (`state_estimator_msgs/ContactDetection`)
- `/state_estimator/leg_odometry` (`state_estimator_msgs/LegOdometry`)
- `/state_estimator/sensor_fusion` (`nav_msgs/Odometry`)
- TF from `world` to `base`, generated from `/state_estimator/sensor_fusion`

The topics and frame IDs are configured in `muse_ws/src/state_estimator/config`.

## :gear: Plugins

The state-estimator node discovers plugins declared in `state_estimator_plugins.xml` and loads them according to `launch/pluginlist.yaml`.

Implemented ROS1 plugins:

- `AttitudeEstimation`: estimates attitude from IMU data.
- `ContactDetection`: estimates foot contact from actuator readings and URDF-based GRF estimation.
- `LegOdometry`: estimates base velocity from actuator readings, attitude, contact state, and Pinocchio kinematics.
- `SensorFusion`: fuses IMU, attitude, and leg odometry into `nav_msgs/Odometry`.
- `TfPublisher`: publishes TF from the fused odometry output.

By default, both `plugin_whitelist` and `plugin_blacklist` are empty, so all declared plugins are loaded. To run only selected plugins, edit `muse_ws/src/state_estimator/launch/pluginlist.yaml`.

### Visualization
Run PlotJuggler with:

```sh
rosrun plotjuggler plotjuggler
```
A PlotJuggler layout is also provided at:

```text
muse_ws/src/plotjuggler_layout.xml
```


## :robot: Robot Configuration

To use a different robot:

1. Add the URDF to `muse_ws/src/state_estimator/urdfs`.
2. Update `urdf_path` in `config/contact_plugin.yaml` and `config/leg_odometry.yaml`.
3. Update `foot_frame_names` and the joint-name lists in the same config files.
4. Update `base_R_imu` and input topic names for your robot.

The default leg order is:

```text
LF, RF, LH, RH
```

Keep this order consistent across contact detection, leg odometry, messages, and downstream consumers.


## :scroll: TODO list

- [x] ROS1 proprioceptive state estimation
- [x] GRF-based contact detection from actuator readings
- [x] TF publishing from fused odometry
- [ ] ROS2 support (on going)
- [ ] Exteroceptive sensor fusion (on going)

## :hugs: Contributing

Contributions to this repository are welcome.

## Citing the paper

If you like this work and would like to cite it (thanks):

```bibtex
@ARTICLE{10933515,
  author={Nisticò, Ylenia and Soares, João Carlos Virgolino and Amatucci, Lorenzo and Fink, Geoff and Semini, Claudio},
  journal={IEEE Robotics and Automation Letters},
  title={MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots},
  year={2025},
  volume={10},
  number={5},
  pages={4620-4627},
  keywords={Robots;Sensors;Robot sensing systems;Legged locomotion;Odometry;Cameras;Laser radar;Robot vision systems;Robot kinematics;Quadrupedal robots;State estimation;localization;sensor fusion;quadruped robots},
  doi={10.1109/LRA.2025.3553047}}
```

## Maintainer
This repo is maintained by

| Avatar | Name |
| ------- | ---- |
| <img src="https://github.com/ylenianistico.png?size=32" width="32" height="32" style="border-radius:50%; vertical-align:middle; margin:0 6px;" /> | <a href="https://github.com/ylenianistico">Ylenia Nisticò</a> |
