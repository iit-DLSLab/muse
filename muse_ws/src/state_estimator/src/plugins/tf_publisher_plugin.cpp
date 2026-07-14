#include "state_estimator/plugin.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace state_estimator_plugins
{

class TfPublisherPlugin : public PluginBase
{
public:
  std::string getName() const override { return "TfPublisher"; }
  std::string getDescription() const override { return "Fused odometry TF publisher"; }

private:
  void initialize_() override
  {
    const auto odom_topic = parameter<std::string>(
      "tf_publisher_plugin.odom_topic", "~/sensor_fusion");
    parent_frame_ = parameter<std::string>(
      "tf_publisher_plugin.parent_frame", "world");
    child_frame_ = parameter<std::string>(
      "tf_publisher_plugin.child_frame", "base");
    use_odom_frame_ids_ = parameter<bool>(
      "tf_publisher_plugin.use_odom_frame_ids", false);

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { odometryCallback(message); });
    RCLCPP_INFO(
      node_->get_logger(), "TfPublisherPlugin broadcasting %s -> %s from %s",
      parent_frame_.c_str(), child_frame_.c_str(), odom_topic.c_str());
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry)
  {
    if (isPaused()) return;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(odometry->header.stamp).nanoseconds() == 0
      ? static_cast<builtin_interfaces::msg::Time>(node_->now()) : odometry->header.stamp;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    if (use_odom_frame_ids_) {
      if (!odometry->header.frame_id.empty()) {
        transform.header.frame_id = odometry->header.frame_id;
      }
      if (!odometry->child_frame_id.empty()) {
        transform.child_frame_id = odometry->child_frame_id;
      }
    }
    transform.transform.translation.x = odometry->pose.pose.position.x;
    transform.transform.translation.y = odometry->pose.pose.position.y;
    transform.transform.translation.z = odometry->pose.pose.position.z;
    transform.transform.rotation = odometry->pose.pose.orientation;
    broadcaster_->sendTransform(transform);
  }

  void shutdown_() override
  {
    odom_sub_.reset();
    broadcaster_.reset();
  }
  void pause_() override {}
  void resume_() override {}
  void reset_() override {}

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::string parent_frame_;
  std::string child_frame_;
  bool use_odom_frame_ids_{false};
};

}  // namespace state_estimator_plugins

PLUGINLIB_EXPORT_CLASS(
  state_estimator_plugins::TfPublisherPlugin,
  state_estimator_plugins::PluginBase)
