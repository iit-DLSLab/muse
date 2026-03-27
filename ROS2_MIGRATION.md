# ROS1 → ROS2 Migration Notes

This repository was originally ROS1 (`catkin`) and has now been moved to a ROS2-oriented layout (`ament` + `colcon`) with a staged migration approach.

## What is migrated in this branch

- Docker workflow removed.
- Conda environment added (`environment.yml`) using RoboStack (ROS2 Humble).
- `state_estimator_msgs` converted to ROS2 message/service generation.
- `iit_commons` converted to `ament_cmake` as a plain C++ support library.
- `state_estimator` package converted to ROS2 metadata/build system with:
  - ROS2 executable stub (`state_estimator_node`)
  - ROS2 launch file (`state_estimator.launch.py`)
  - installation of config/launch/urdf/plugin xml assets

## Remaining migration work

The full ROS1 runtime behavior is not yet ported. Remaining items include:

1. Port ROS API in all node/plugin sources:
   - `ros::NodeHandle` → `rclcpp::Node`
   - `ros::Publisher/Subscriber/ServiceServer` → ROS2 publisher/subscription/service APIs
   - `ros::Time::now()` → `node->get_clock()->now()`
2. Port service callback signatures in `state_estimator_node`.
3. Port plugin loading and XML exports to ROS2 conventions (`pluginlib_export_plugin_description_file`).
4. Replace ROS1 launch XML assumptions with ROS2 parameters/namespaces as needed.
5. Re-enable full plugin libraries in `state_estimator/CMakeLists.txt` after source porting.

## Recommended workflow

```bash
conda env create -f environment.yml
conda activate muse-ros2
cd muse_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch state_estimator state_estimator.launch.py
```

If you need parity with ROS1 behavior immediately, use the upstream `ros2` branch as reference and cherry-pick missing source changes into this branch.
