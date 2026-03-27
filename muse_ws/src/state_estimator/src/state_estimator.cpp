#include "state_estimator/state_estimator_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("state_estimator");
  RCLCPP_INFO(node->get_logger(), "Starting state estimator");

  state_estimator::state_estimator_node se_node(node);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  se_node.shutdown();
  rclcpp::shutdown();
  return 0;
}

