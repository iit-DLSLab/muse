<h1 align="center"> MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots </h1>
<h3 align="center">Ylenia Nisticò, João Carlos Virgolino Soares, Lorenzo Amatucci, Geoff Fink and Claudio Semini</h3>	

<h4 align="center">This paper has been accepted to IEEE Robotics and Automation Letters, and it is available at https://arxiv.org/abs/2503.12101 </h4>

<h3 align="center"> 
    
![muse_cropped](https://github.com/user-attachments/assets/b212edff-44a4-4e46-acb9-c48e160ae8cd)
    

# :computer: Code

The `muse` package provides a ROS node and utilities for estimating the state of a quadruped robot using sensor data. It includes algorithms for state estimation, sensor fusion, and filtering.

This first version of the code provides a proprioceptive state estimator for quadruped robots. The necessary inputs are 
- **imu measurements**
- **joint states**
- **force exerted on the feet**

    
Additional code to fuse exteroceptive measurements will be available soon!
TODO list at the end of the page
</h2>



## :t-rex: Prerequisites
* [Conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio/tree/master)

⚠️ ATTENTION: ROS1 is deprecated. This repository is being migrated to ROS2 (`ament` + `colcon`) with a staged approach.



## :hammer_and_wrench: Building and Running

To install and run `muse` with Conda + ROS2:

1. Clone and create the environment:
  ```sh
  git clone https://github.com/iit-DLSLab/muse.git
  cd muse
  conda env create -f environment.yml
  conda activate muse-ros2
  ```

2. Build with `colcon`:
  ```sh
  cd muse_ws
  colcon build --symlink-install
  source install/setup.bash
  ```

3. Launch the state estimator package:
   ```sh
   ros2 launch state_estimator state_estimator.launch.py
   ```

For migration details and remaining ROS1→ROS2 source porting tasks, see `ROS2_MIGRATION.md`.
To change the name of the topics, check the [config foder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/config).

To visualize your data, you can use [PlotJuggler](https://github.com/facontidavide/PlotJuggler?tab=readme-ov-file):
```sh
ros2 run plotjuggler plotjuggler
```

:warning: In this repo we provide an example with the ANYmal B300 robot. If you want to test MUSE with another one, you only need to add the URDF of your robot in [this folder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/urdfs), and change the name of the legs in the [leg odometry plugin, line 249](https://github.com/iit-DLSLab/muse/blob/main/muse_ws/src/state_estimator/src/plugins/leg_odometry_plugin.cpp#L249):

``` sh
std::vector<std::string> feet_frame_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};   // Update with your actual link names
```
For real-world experiments, we recommend using this very nice [MPC](https://github.com/iit-DLSLab/Quadruped-PyMPC) to control your robot!
## :scroll: TODO list
- [ ] Extend the code to include exteroception
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
This repo is maintained by [Ylenia Nisticò](https://github.com/ylenianistico)




