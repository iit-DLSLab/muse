#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("state_estimator");

  RCLCPP_WARN(
    node->get_logger(),
    "ROS2 migration in progress: running state_estimator ROS2 stub node. "
    "Port plugin sources from ROS1 APIs to ROS2 APIs to enable full runtime behavior.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
