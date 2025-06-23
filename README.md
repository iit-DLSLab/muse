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
* [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) (but it should be compatible with the other ROS versions)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio/tree/master)

⚠️ Don't worry! In this repo, we provide **Dockerization** to avoid dealing with the dependencies!

## :hammer_and_wrench: Building and Running

To install the `muse` package, follow these steps:

1. Clone this repository and build the Docker image:
    ```sh
    git clone https://github.com/iit-DLSLab/muse.git
    cd muse
    docker build -t muse-docker .
    ```

2. Enter the docker and build using `catkin_make`:
    ```sh
    cd muse_ws
    xhost +local:docker
    docker run -it --rm --name muse -v "$(pwd)":/root/muse_ws -w  /root/muse_ws muse-docker
    catkin_make -j$(proc) install
    source devel/setup.bash  
    ```
3. To launch the state estimator node:
   ```sh
   roslaunch state_estimator state_estimator.launch
   ```
If you need to read the data from a rosbag, you need to mount the folder where you store your rosbags (`your_path_to_rosbags`), to make it visible inside the image, and then, you can attach a docker image in another terminal, for example:
```sh
docker run -it --rm --name muse \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v your_path_to_rosbags:/root/rosbags \
  -v "$(pwd)":/root/muse_ws \
  -w /root/muse_ws \
  muse-docker (terminal 1)
docker exec -it muse bash (terminal 2)
source devel/setup.bash
cd ~/rosbags (terminal 2)
rosbag play your_rosbag.bag (terminal 2)
```
To change the name of the topics, check the [config foder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/config).

To visualize your data, you can use [PlotJuggler](https://github.com/facontidavide/PlotJuggler?tab=readme-ov-file) which is already installed in the docker image:
```sh
rosrun plotjuggler plotjuggler
```

:warning: In this repo we provide an example with the ANYmal B300 robot. If you want to test MUSE with another one, you only need to add the URDF of your robot in [this folder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/urdfs), and change the name of the legs in the [leg odometry plugin, line 249](https://github.com/iit-DLSLab/muse/blob/main/muse_ws/src/state_estimator/src/plugins/leg_odometry_plugin.cpp#L249):

``` sh
std::vector<std::string> feet_frame_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};   // Update with your actual link names
```
For real-world experiments, we recommend using this very nice [MPC](https://github.com/iit-DLSLab/Quadruped-PyMPC) to control your robot!
## :scroll: TODO list
- [ ] Extend the code to include exteroception
- [x] Dockerization
- [ ] Support for ROS2

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




