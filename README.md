<h2 align="center"> MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots </h1>
<h4 align="center">Ylenia Nisticò, João Carlos Virgolino Soares, Lorenzo Amatucci, Geoff Fink and Claudio Semini</h4>

<h4 align="center">IEEE Robotics and Automation Letters · <a href="https://arxiv.org/abs/2503.12101">Paper</a></h4>

> **Branch note (`unitree_sdk`)**: this branch is developed for full compatibility with the Unitree SDK and ROS2 communication. The code has been tested on the Unitree Go2 and A2 robots in both simulation and real-world experiments.

<h3 align="center"> 
    
<!-- ![muse_cropped](https://github.com/user-attachments/assets/b212edff-44a4-4e46-acb9-c48e160ae8cd) -->
    

# :computer: Overview

MUSE estimates quadruped robot state from **IMU measurements** and **joint states** using ROS2. This branch provides proprioceptive state estimation, with **Pixi** (recommended, locked dependencies) and **Conda** environments.

## :octocat: Suggestions

- **Simulation:** [basic-locomotion-dls-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-dls-isaaclab).
- **Real robots:** To run this code on Unitree robots connect through [unitree_ros2_dls](https://github.com/iit-DLSLab/unitree_ros2_dls). For control, we recommend [basic-locomotion-dls-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-dls-isaaclab) or [Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC).

## :t-rex: Prerequisites

Linux x86-64 and [Pixi](https://pixi.sh/latest/installation/) or [Conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html). Both environments include ROS2 Humble, [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page), [Pinocchio](https://github.com/stack-of-tasks/pinocchio/tree/master), [vcstool](https://github.com/dirk-thomas/vcstool), and the C++ toolchain.

## :hammer_and_wrench: Building and Running

Clone the repository:

```sh
git clone https://github.com/iit-DLSLab/muse.git -b unitree_sdk
cd muse
```

### Pixi (recommended)

From the repository root, in a terminal without another Conda or ROS environment activated:

```sh
pixi install --locked
pixi run --locked build
pixi run --locked launch
```

Tasks source ROS and the workspace automatically. `--locked` uses the committed dependency versions and rejects manifest/lockfile mismatches. To clean build artifacts, run `pixi run --locked clean`.

Alternatively, build and launch manually in a Pixi shell, starting from the repository root:

```sh
pixi install --locked
pixi shell --locked
source "$CONDA_PREFIX/setup.bash"
cd muse_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch state_estimator state_estimator.launch.py
```

Skip `colcon build` if the workspace is already built. Use `exit` to leave the Pixi shell.

### Conda

```sh
conda env create -f environment.yml
conda activate muse-ros2
source "$CONDA_PREFIX/setup.bash"
cd muse_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch state_estimator state_estimator.launch.py
```

### Build options and dependencies

Fresh builds default to `Release`. To switch to `Debug`, clean first to clear CMake's cached configuration:

```sh
pixi run --locked clean
pixi run --locked build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Use `Release` to switch back; with Conda, pass the same `--cmake-args` to `colcon build`.

To update dependencies, edit `pixi.toml`, keep shared constraints aligned with `environment.yml`, run `pixi lock`, then install, clean, and rebuild. Commit `pixi.toml` and `pixi.lock` together. Both environments pin Python 3.11, CMake 3.26, GCC/G++ 14, and Empy 3 for ROS 2 Humble compatibility.

## Robot configuration and checks

The default configuration uses **A2** with `unitree_hg/msg/LowState` on `/lowstate`. Select your robot and input topic in [robot.yaml](muse_ws/src/state_estimator/config/robot.yaml); adjust output topics in the [config folder](muse_ws/src/state_estimator/config). For another robot, add its [URDF](muse_ws/src/state_estimator/urdfs) and configure joint mappings and foot frames.

With the estimator running and compatible robot data or a ROS 2 bag, inspect output from a separate terminal. For Pixi, start at the repository root:

```sh
pixi shell --locked
source "$CONDA_PREFIX/setup.bash"
source muse_ws/install/setup.bash
ros2 node info /state_estimator
ros2 topic hz /base_state
ros2 topic echo /muse/proprioceptive_sensor_fusion --once
```

From a sourced environment, run `colcon test --return-code-on-test-failure` and `colcon test-result --verbose` in `muse_ws`. The `iit_commons` numerical tests are currently disabled, so passing tests alone do not validate estimator accuracy.

Visualize data with [PlotJuggler](https://github.com/facontidavide/PlotJuggler):

```sh
ros2 run plotjuggler plotjuggler
```

## :scroll: TODO list

- [ ] Extend the code to include exteroception 
- [x] Conda-based environment
- [x] Full Support for ROS2 (on going)

## :hugs: Contributing

Contributions to this repository are welcome.

## Citing the paper

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

| Avatar | Name |
| ------- | ---- |
| <img src="https://github.com/ylenianistico.png?size=32" width="32" height="32" style="border-radius:50%; vertical-align:middle; margin:0 6px;" /> | <a href="https://github.com/ylenianistico">Ylenia Nisticò</a> |
