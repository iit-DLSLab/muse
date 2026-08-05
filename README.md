# MUSE: Multi-Sensor State Estimator

MUSE is a ROS 2 Humble proprioceptive state estimator for quadruped robots. It
loads estimator modules with `pluginlib` and publishes attitude, contact state,
leg odometry, fused odometry, and TF. The estimator is described in the 2025
IEEE Robotics and Automation Letters paper
[MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots](https://arxiv.org/abs/2503.12101).

> **Validation status**
>
> This ROS 2 port has been successfully built and launched on ROS 2 Humble. The estimator node, plugins, parameters, topics, and services have been checked at startup.
>
> However, the complete estimation pipeline has not yet been validated with real robot data or a ROS 2 bag, and numerical equivalence with the ROS 1 implementation on `main` has not yet been confirmed.

## :t-rex: Prerequisites

- Conda or Mamba
- Dependencies from the provided `environment.yml`

The conda environment is named `muse-ros2` and provides the ROS2 Humble stack, `catkin_tools`, `Eigen`, `Pinocchio`, `RViz`, `PlotJuggler`, and the `ROS2` packages used by the workspace.

Create the environment with:

```sh
cd muse
mamba env create -f environment.yml
```

If the environment already exists, update it with:

```sh
mamba env update -n muse-ros2 -f environment.yml --prune
```

## :earth_africa: Environment

Create or update the Humble environment:

```bash
mamba env create -f environment.yml
# For an existing environment:
mamba env update -n muse-ros2 -f environment.yml --prune
conda activate muse-ros2
```

The environment contains ROS 2 Humble, Eigen, Pinocchio, colcon, RViz2,
PlotJuggler, and rosbag2. MUSE does not require a robot vendor SDK.

## :hammer_and_wrench: Build

```bash
conda activate muse-ros2
cd muse_ws
colcon build --symlink-install
source install/setup.bash
```

<!-- Run automated tests with:

```bash
colcon test
colcon test-result --verbose
``` -->

## :rocket: Launch

```bash
ros2 launch state_estimator state_estimator.launch.py
```

Optional launch arguments are:

```bash
ros2 launch state_estimator state_estimator.launch.py use_sim_time:=true rviz:=true
```

The node remains active while inputs are unavailable and reports missing input
conditions with throttled warnings.

## Inputs and outputs

Default inputs are:

- `/imu` (`sensor_msgs/msg/Imu`)
- `/actuator_state` (`state_estimator_msgs/msg/JointStateWithAcceleration`)

The actuator message contains a header plus joint names, positions, velocities,
accelerations, and efforts. Joint names are mapped to the configured URDF joints;
the estimator does not depend on array ordering when names are present.

Default outputs are:

- `/state_estimator/attitude` (`state_estimator_msgs/msg/Attitude`)
- `/state_estimator/contact_detection` (`state_estimator_msgs/msg/ContactDetection`)
- `/state_estimator/leg_odometry` (`state_estimator_msgs/msg/LegOdometry`)
- `/state_estimator/sensor_fusion` (`nav_msgs/msg/Odometry`)
- `/tf`, normally publishing `world` to `base`

Topics are parameters and may be remapped or changed in the files under
`state_estimator/config`.

## Plugins and services

The node loads the five classes declared in `state_estimator_plugins.xml`:

- `AttitudeEstimation`
- `ContactDetection`
- `LegOdometry`
- `SensorFusion`
- `TfPublisher`

`launch/pluginlist.yaml` provides glob-style `plugin_whitelist` and
`plugin_blacklist` parameters. Empty lists load every plugin. A non-empty
whitelist with an empty blacklist loads only matching whitelist entries.

Private ROS 2 services under `/state_estimator` list and describe plugins and
perform start, stop, pause, resume, restart, and reset operations. For example:

```bash
ros2 service call /state_estimator/list_all_estimators \
  state_estimator_msgs/srv/ListAllEstimators '{}'
ros2 service call /state_estimator/pause_estimator \
  state_estimator_msgs/srv/PauseEstimator '{name: AttitudeEstimation}'
```

## Robot configuration

Contact detection and leg odometry use Pinocchio models loaded from a URDF.
Relative `urdf_path` values are resolved from the installed `state_estimator`
package share directory, so installed launches do not depend on the source tree.
Absolute paths are also accepted.

To configure another robot:

1. Install its URDF with the package or set an absolute `urdf_path`.
2. Set four foot frame names in LF, RF, LH, RH order.
3. Set the corresponding twelve joint names.
4. Set the base frame and IMU-to-base rotation.
5. Tune GRF thresholds and contact options for the actuator effort convention.

Example URDFs for Aliengo, ANYmal, and Go1 naming are retained as configuration
examples only. No vendor messages or SDKs are used.

## rosbag2 playback

Record or replay the generic input topics with rosbag2:

```bash
ros2 launch state_estimator state_estimator.launch.py use_sim_time:=true
ros2 bag play --clock your_bag
```

Older bags containing vendor-specific actuator messages must first be converted
or bridged to `JointStateWithAcceleration`.

## Known limitations

- The current estimator is proprioceptive; it does not fuse exteroceptive odometry.
- The former timeout configuration was inactive and is not part of the ROS 2 node.

## Citation

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
  doi={10.1109/LRA.2025.3553047}}
```
