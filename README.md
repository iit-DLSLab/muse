<h1 align="center"> MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots </h1>
<h3 align="center">Ylenia Nisticò, João Carlos Virgolino Soares, Lorenzo Amatucci, Geoff Fink and Claudio Semini</h3>	

<h4 align="center">This paper has been accepted to IEEE Robotics and Automation Letters, and it is available at https://arxiv.org/abs/2503.12101 </h4>

> **Branch note (`unitree_sdk`)**: this branch is developed for full compatibility with the Unitree SDK and ROS2 communication. The code has been tested on a Unitree Go2 robot in both simulation and real-world experiments.

<h3 align="center"> 
    
<!-- ![muse_cropped](https://github.com/user-attachments/assets/b212edff-44a4-4e46-acb9-c48e160ae8cd) -->
    

# :computer: Overview

The `muse` package provides a ROS node and utilities for estimating the state of a quadruped robot using sensor data. It includes algorithms for state estimation, sensor fusion, and filtering.

This version of the code provides a proprioceptive state estimator for quadruped robots. The necessary inputs are 
- **imu measurements**
- **joint states**
- **force exerted on the feet**

### To simplify setup, we provide a ready-to-use **Conda** environment so you do not have to manually resolve dependencies.


## :octocat: Suggestions
### Simulation
If you want a running simulation, follow the instructions in [basic-locomotion-dls-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-dls-isaaclab).

### Real world experiments
To run this code with Unitree robots, you need to port the URDF of your robot in [this folder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/urdfs) and connect to the robot through [unitree_ros2_dls](https://github.com/iit-DLSLab/unitree_ros2_dls).

For real-world experiments, we recommend the following repositories to control your robot:
- [basic-locomotion-dls-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-dls-isaaclab)
- [Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC)

## :t-rex: Prerequisites
* [Conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio/tree/master)
* [vcstool](https://github.com/dirk-thomas/vcstool)


## :hammer_and_wrench: Building and Running

To install and run `muse` with Conda + ROS2:

1. Clone and create the environment:
    ```sh
    git clone https://github.com/iit-DLSLab/muse.git -b unitree_sdk
    cd muse
    conda env create -f environment.yml
    conda activate muse-ros2
    ```

2. Fetch external dependencies (e.g. Point-LIO):
    ```sh
    cd muse_ws
    vcs import src < muse.repos
    ```

3. Build with `colcon`:
    ```sh
    cd muse_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

4. Launch the state estimator package:
    ```sh
    ros2 launch state_estimator state_estimator.launch.py
    ```

    Or launch the full stack with Point-LIO (see [Point-LIO integration](#lidar-point-lio-integration)):
    ```sh
    ros2 launch muse_point_lio muse_with_point_lio.launch.py
    ```

To change the name of the topics, check the [config folder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/config).

To visualize your data, you can use [PlotJuggler](https://github.com/facontidavide/PlotJuggler?tab=readme-ov-file):
```sh
ros2 run plotjuggler plotjuggler
```

---

## :satellite: LiDAR / Point-LIO Integration

MUSE includes a `muse_point_lio` wrapper package that integrates [Point-LIO ROS2](https://github.com/ylenianistico/point_lio_ros2/tree/muse-integration) as the exteroceptive odometry source for the `MultiSensorFusion` plugin.

### Architecture
```
[Point-LIO node]  →  /point_lio/odometry
        ↓
[odom_bridge node]  →  /lidar_odometry     (normalised interface)
        ↓
[MultiSensorFusion plugin]  →  /muse/multi_sensor_fusion
        ↓
[TfStatePublisher plugin]  →  TF: world → base
```

### Go2-specific setup
A ready-to-use Point-LIO config for the Go2 is provided at [`config/go2_muse.yaml`](https://github.com/ylenianistico/point_lio_ros2/blob/muse-integration/config/go2_muse.yaml) in the `muse-integration` branch of the forked Point-LIO repo. It sets:
- Input topics: `/utlidar/cloud` and `/utlidar/imu` (Unitree SDK2 native driver)
- Extrinsics derived from the Go2 URDF (`radar_joint` → `imu_joint`)
- Frame IDs: `odom_header_frame_id: odom`, `odom_child_frame_id: imu`

The Point-LIO fork is fetched automatically via `muse.repos` (see build instructions above).

### Selecting a different LiDAR
Pass `point_lio_launch_file` to switch to any other Point-LIO launch file:
```sh
ros2 launch muse_point_lio muse_with_point_lio.launch.py \
  point_lio_launch_file:=mapping_velody16.launch.py
```

---

:warning: In this repo we provide an example with the Go2 robot. If you want to test MUSE with another one, you need to add the URDF of your robot in [this folder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/urdfs), and (possibly) change the name of the legs in the [config files](https://github.com/iit-DLSLab/muse/blob/main/muse_ws/src/state_estimator/config)


## :scroll: TODO list
- [x] Extend the code to include exteroception (Point-LIO integration via `muse_point_lio`)
- [x] Conda-based environment
- [x] Support for ROS2 (on going)

## :hugs: Contributing

Contributions to this repository are welcome.

## Citing the paper

If you like this work and would like to cite it (thanks):
```
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
<p>
    This repo is maintained by
    
  | <img src="https://github.com/ylenianistico.png?size=32" width="32" height="32" style="border-radius:50%; vertical-align:middle; margin:0 6px;" /> | <a href="https://github.com/ylenianistico">Ylenia Nisticò</a> |
  | -------| ----------- |
  
</p>



