#include <cstdint>

#include "state_estimator/Models/robot_config.hpp"
#include "state_estimator/Models/unitree_low_state_adapter.hpp"
#include "state_estimator/plugin.hpp"

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class TfStatePublisherPlugin : public PluginBase
{
public:
  std::string getName() override { return std::string("TfStatePublisher"); }
  std::string getDescription() override { return std::string("Publishes world->base TF and joint_states"); }

  void initialize_() override
  {
    state_estimator::RobotConfig robot_config;
    std::string config_error;
    if (!state_estimator::loadRobotConfig(config_dir_, robot_config, config_error)) {
      throw std::runtime_error("TfStatePublisherPlugin: " + config_error);
    }

    robot_type_ = robot_config.robot_type;
    odom_topic_ = "muse/proprioceptive_sensor_fusion";
    low_state_topic_ = robot_config.low_state_topic;
    joint_states_topic_ = "joint_states";
    world_frame_id_ = "world";
    base_frame_id_ = "base";
    lidar_frame_id_ = "radar";
    lidar_imu_frame_id_ = "radar_imu";
    publish_base_tf_ = true;
    publish_joint_states_ = true;
    publish_lidar_imu_tf_ = true;
    motor_joint_names_ = robot_config.motor_joint_names;
    motor_indices_ = robot_config.motor_indices;

    if (!config_dir_.empty()) {
      try {
        YAML::Node cfg = YAML::LoadFile(config_dir_ + "/tf_state_publisher.yaml")["tf_state_publisher_plugin"];
        if (cfg["odom_topic"]) odom_topic_ = cfg["odom_topic"].as<std::string>();
        if (cfg["joint_states_topic"]) joint_states_topic_ = cfg["joint_states_topic"].as<std::string>();
        if (cfg["world_frame_id"]) world_frame_id_ = cfg["world_frame_id"].as<std::string>();
        if (cfg["base_frame_id"]) base_frame_id_ = cfg["base_frame_id"].as<std::string>();
        if (cfg["lidar_frame_id"]) lidar_frame_id_ = cfg["lidar_frame_id"].as<std::string>();
        if (cfg["lidar_imu_frame_id"]) lidar_imu_frame_id_ = cfg["lidar_imu_frame_id"].as<std::string>();
        if (cfg["publish_base_tf"]) publish_base_tf_ = cfg["publish_base_tf"].as<bool>();
        if (cfg["publish_joint_states"]) publish_joint_states_ = cfg["publish_joint_states"].as<bool>();
        if (cfg["publish_lidar_imu_tf"]) publish_lidar_imu_tf_ = cfg["publish_lidar_imu_tf"].as<bool>();
      } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Could not load tf_state_publisher config: %s", e.what());
      }
    }

    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 250);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 250,
      std::bind(&TfStatePublisherPlugin::callback_odometry, this, std::placeholders::_1));

    createLowStateSubscription();

    if (publish_lidar_imu_tf_) {
      geometry_msgs::msg::TransformStamped lidar_imu_tf;
      lidar_imu_tf.header.stamp = node_->now();
      lidar_imu_tf.header.frame_id = lidar_frame_id_;
      lidar_imu_tf.child_frame_id = lidar_imu_frame_id_;
      lidar_imu_tf.transform.rotation.w = 1.0;
      static_tf_broadcaster_->sendTransform(lidar_imu_tf);
    }

    RCLCPP_INFO(node_->get_logger(),
      "TfStatePublisherPlugin initialized (odom: %s, lowstate: %s, robot_type: %s)",
      odom_topic_.c_str(), low_state_topic_.c_str(), robot_type_.c_str());
  }

  void shutdown_() override {}
  void pause_() override {}
  void resume_() override {}
  void reset_() override {}

private:
  void createLowStateSubscription()
  {
    const auto family = state_estimator::messageFamilyForRobotType(robot_type_);
    if (family == state_estimator::UnitreeMessageFamily::Go) {
      go_low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
        low_state_topic_, 250,
        std::bind(&TfStatePublisherPlugin::callbackGo, this, std::placeholders::_1));
      return;
    }

    hg_low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
      low_state_topic_, 250,
      std::bind(&TfStatePublisherPlugin::callbackHg, this, std::placeholders::_1));
  }

  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    if (!publish_base_tf_) return;

    geometry_msgs::msg::TransformStamped base_tf;
    base_tf.header.stamp = odom->header.stamp;
    base_tf.header.frame_id = odom->header.frame_id.empty() ? world_frame_id_ : odom->header.frame_id;
    base_tf.child_frame_id = odom->child_frame_id.empty() ? base_frame_id_ : odom->child_frame_id;
    base_tf.transform.translation.x = odom->pose.pose.position.x;
    base_tf.transform.translation.y = odom->pose.pose.position.y;
    base_tf.transform.translation.z = odom->pose.pose.position.z;
    base_tf.transform.rotation = odom->pose.pose.orientation;
    tf_broadcaster_->sendTransform(base_tf);
  }

  void callbackGo(const unitree_go::msg::LowState::SharedPtr low_state)
  {
    std::string error;
    if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "TfStatePublisherPlugin: failed to adapt unitree_go LowState: %s", error.c_str());
      return;
    }
    processLowState(normalized_low_state_);
  }

  void callbackHg(const unitree_hg::msg::LowState::SharedPtr low_state)
  {
    std::string error;
    if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "TfStatePublisherPlugin: failed to adapt unitree_hg LowState: %s", error.c_str());
      return;
    }
    processLowState(normalized_low_state_);
  }

  void processLowState(const state_estimator::UnitreeLowState& low_state)
  {
    if (!publish_joint_states_) return;

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = node_->now();
    joint_state_msg.name = motor_joint_names_;
    joint_state_msg.position.resize(motor_joint_names_.size(), 0.0);
    joint_state_msg.velocity.resize(motor_joint_names_.size(), 0.0);
    joint_state_msg.effort.resize(motor_joint_names_.size(), 0.0);

    for (std::size_t i = 0; i < motor_joint_names_.size(); ++i) {
      const std::size_t motor_index = motor_indices_[i];
      if (motor_index >= low_state.motors.size()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "TfStatePublisherPlugin: configured motor index %zu is outside LowState motor size %zu",
          motor_index, low_state.motors.size());
        return;
      }
      joint_state_msg.position[i] = low_state.motors[motor_index].q;
      joint_state_msg.velocity[i] = low_state.motors[motor_index].dq;
      joint_state_msg.effort[i] = low_state.motors[motor_index].tau_est;
    }

    joint_state_pub_->publish(joint_state_msg);
  }

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr go_low_state_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr hg_low_state_sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  state_estimator::UnitreeLowState normalized_low_state_;
  std::string robot_type_;
  std::string odom_topic_;
  std::string low_state_topic_;
  std::string joint_states_topic_;
  std::string world_frame_id_;
  std::string base_frame_id_;
  std::string lidar_frame_id_;
  std::string lidar_imu_frame_id_;
  bool publish_base_tf_{true};
  bool publish_joint_states_{true};
  bool publish_lidar_imu_tf_{true};
  std::vector<std::string> motor_joint_names_;
  std::vector<std::size_t> motor_indices_;
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::TfStatePublisherPlugin, state_estimator_plugins::PluginBase)
