#include "state_estimator/state_estimator_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("state_estimator");
  RCLCPP_INFO(node->get_logger(), "Starting state estimator");

  state_estimator::StateEstimatorNode estimator(node);
  rclcpp::spin(node);
  estimator.shutdown();
  rclcpp::shutdown();
  return 0;
}
