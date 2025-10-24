#include "state_estimator/plugin.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace state_estimator_plugins
{

class JointStateDownsamplerPlugin : public PluginBase
{
public:
  JointStateDownsamplerPlugin() = default;
  ~JointStateDownsamplerPlugin() override = default;

  std::string getName() override { return std::string("JointStateDownsampler"); }
  std::string getDescription() override { return std::string("Downsamples joint_states to a configurable rate"); }

  void initialize_() override
  {
    auto nh = this->node_;
    if (!nh) {
      throw std::runtime_error("JointStateDownsamplerPlugin: node_ is null");
    }

    // Parameters (ROS 2 style). Provide legacy ROS 1-style backups for compatibility.
    input_topic_ = nh->declare_parameter<std::string>(
        "joint_state_downsampler.input_topic", "joint_states");
    output_topic_ = nh->declare_parameter<std::string>(
        "joint_state_downsampler.output_topic", "/state_estimator/joint_states_downsampled");
    downsampling_rate_ = nh->declare_parameter<int>(
        "joint_state_downsampler.downsampling_rate", 4);

    // Interfaces
    using sensor_msgs::msg::JointState;
    sub_ = nh->create_subscription<JointState>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&JointStateDownsamplerPlugin::onJointState, this, std::placeholders::_1));
    pub_ = nh->create_publisher<JointState>(output_topic_, rclcpp::SensorDataQoS());

    index = 0;

    RCLCPP_INFO(nh->get_logger(), "JointStateDownsamplerPlugin listening on '%s', downsampling '%s' by %d",
                input_topic_.c_str(), output_topic_.c_str(), downsampling_rate_);
  }

  void shutdown_() override {}
  void pause_() override {}
  void resume_() override {}
  void reset_() override { index = 0; }

private:
  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    
    // Publish first immediately or if period elapsed.
    if (index % downsampling_rate_ == 0) {
      pub_->publish(*msg);
      index = 1;
    } else {
      index++;
    }
  }

  // Parameters
  std::string input_topic_{};
  std::string output_topic_{};
  int downsampling_rate_{};
  rclcpp::Duration period_{0, 0};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

  // State
  int index;
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::JointStateDownsamplerPlugin, state_estimator_plugins::PluginBase)
