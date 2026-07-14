#include "state_estimator/Models/grf_contact_estimator.hpp"
#include "state_estimator/Models/joint_state_snapshot.hpp"
#include "state_estimator/plugin.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <state_estimator_msgs/msg/contact_detection.hpp>
#include <state_estimator_msgs/msg/joint_state_with_acceleration.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class ContactDetectionPlugin : public PluginBase
{
public:
  std::string getName() const override { return "ContactDetection"; }
  std::string getDescription() const override
  {
    return "URDF and inverse-dynamics GRF contact detection plugin";
  }

private:
  void initialize_() override
  {
    const auto pub_topic = parameter<std::string>(
      "contact_detection_plugin.pub_topic", "~/contact_detection");
    const auto actuator_topic = parameter<std::string>(
      "contact_detection_plugin.actuator_state_topic", "/actuator_state");
    const auto urdf_path = resolveResource(parameter<std::string>(
      "contact_detection_plugin.urdf_path", "urdfs/anymal.urdf"));

    const double threshold = parameter<double>(
      "contact_detection_plugin.grf_threshold", 40.0);
    const double threshold_low = parameter<double>(
      "contact_detection_plugin.grf_threshold_low", threshold);
    const double threshold_high = parameter<double>(
      "contact_detection_plugin.grf_threshold_high", threshold);
    const bool use_hysteresis = parameter<bool>(
      "contact_detection_plugin.use_hysteresis", false);
    const bool use_force_norm = parameter<bool>(
      "contact_detection_plugin.use_force_norm", false);
    const bool use_absolute_normal_force = parameter<bool>(
      "contact_detection_plugin.use_absolute_normal_force", true);
    const bool use_inverse_dynamics_compensation = parameter<bool>(
      "contact_detection_plugin.use_inverse_dynamics_compensation", true);
    const double force_sign = parameter<double>(
      "contact_detection_plugin.force_sign", -1.0);

    const auto feet = toStringArray4(
      parameter<std::vector<std::string>>(
        "contact_detection_plugin.foot_frame_names",
        {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}),
      {{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}},
      "contact_detection_plugin.foot_frame_names");
    const std::array<std::vector<std::string>, 4> leg_joints{{
      parameter<std::vector<std::string>>(
        "contact_detection_plugin.lf_joint_names", {"LF_HAA", "LF_HFE", "LF_KFE"}),
      parameter<std::vector<std::string>>(
        "contact_detection_plugin.rf_joint_names", {"RF_HAA", "RF_HFE", "RF_KFE"}),
      parameter<std::vector<std::string>>(
        "contact_detection_plugin.lh_joint_names", {"LH_HAA", "LH_HFE", "LH_KFE"}),
      parameter<std::vector<std::string>>(
        "contact_detection_plugin.rh_joint_names", {"RH_HAA", "RH_HFE", "RH_KFE"})
    }};
    const Eigen::Vector3d contact_normal = toVector3(
      parameter<std::vector<double>>(
        "contact_detection_plugin.contact_normal", {0.0, 0.0, 1.0}),
      Eigen::Vector3d::UnitZ(), "contact_detection_plugin.contact_normal");

    try {
      if (!estimator_.configure(
          urdf_path, feet, leg_joints, threshold, threshold_low, threshold_high,
          use_hysteresis, use_force_norm, use_absolute_normal_force, force_sign,
          use_inverse_dynamics_compensation, contact_normal)) {
        throw std::runtime_error(estimator_.lastError());
      }
    } catch (const std::exception& error) {
      throw std::runtime_error(
        std::string("failed to configure GRF contact estimator: ") + error.what());
    }

    pub_ = node_->create_publisher<state_estimator_msgs::msg::ContactDetection>(
      pub_topic, rclcpp::QoS(250));
    actuator_sub_ = node_->create_subscription<
      state_estimator_msgs::msg::JointStateWithAcceleration>(
      actuator_topic, rclcpp::SensorDataQoS(),
      [this](state_estimator_msgs::msg::JointStateWithAcceleration::ConstSharedPtr message) {
        actuatorCallback(message);
      });

    RCLCPP_INFO(
      node_->get_logger(), "ContactDetectionPlugin reading %s with URDF %s",
      actuator_topic.c_str(), urdf_path.c_str());
  }

  void actuatorCallback(
    const state_estimator_msgs::msg::JointStateWithAcceleration::ConstSharedPtr& message)
  {
    if (isPaused()) return;

    state_estimator::JointStateSnapshot joints;
    joints.name = message->name;
    joints.position = message->position;
    joints.velocity = message->velocity;
    joints.acceleration = message->acceleration;
    joints.effort = message->effort;

    state_estimator::GrfContactEstimate estimate;
    if (!estimator_.update(joints, estimate)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "GRF contact update failed: %s", estimator_.lastError().c_str());
      return;
    }

    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Contact metrics [LF RF LH RH]: %.3f %.3f %.3f %.3f",
      estimate.metric[0], estimate.metric[1], estimate.metric[2], estimate.metric[3]);

    state_estimator_msgs::msg::ContactDetection output;
    output.header.stamp = stampOrNow(message->header.stamp);
    output.stance_lf = estimate.stance[0];
    output.stance_rf = estimate.stance[1];
    output.stance_lh = estimate.stance[2];
    output.stance_rh = estimate.stance[3];
    pub_->publish(output);
  }

  builtin_interfaces::msg::Time stampOrNow(const builtin_interfaces::msg::Time& stamp) const
  {
    return rclcpp::Time(stamp).nanoseconds() == 0
      ? static_cast<builtin_interfaces::msg::Time>(node_->now()) : stamp;
  }

  static std::string resolveResource(const std::string& path)
  {
    const std::filesystem::path requested(path);
    if (requested.is_absolute()) return requested.string();
    return (std::filesystem::path(
      ament_index_cpp::get_package_share_directory("state_estimator")) / requested).string();
  }

  std::array<std::string, 4> toStringArray4(
    const std::vector<std::string>& values,
    const std::array<std::string, 4>& fallback,
    const std::string& parameter_name) const
  {
    if (values.size() == 4) return {{values[0], values[1], values[2], values[3]}};
    RCLCPP_WARN(
      node_->get_logger(), "%s must contain four names; using defaults",
      parameter_name.c_str());
    return fallback;
  }

  Eigen::Vector3d toVector3(
    const std::vector<double>& values,
    const Eigen::Vector3d& fallback,
    const std::string& parameter_name) const
  {
    if (values.size() == 3) return {values[0], values[1], values[2]};
    RCLCPP_WARN(
      node_->get_logger(), "%s must contain three values; using default",
      parameter_name.c_str());
    return fallback;
  }

  void shutdown_() override
  {
    actuator_sub_.reset();
    pub_.reset();
  }
  void pause_() override {}
  void resume_() override {}
  void reset_() override {}

  rclcpp::Subscription<
    state_estimator_msgs::msg::JointStateWithAcceleration>::SharedPtr actuator_sub_;
  rclcpp::Publisher<state_estimator_msgs::msg::ContactDetection>::SharedPtr pub_;
  state_estimator::GrfContactEstimator estimator_;
};

}  // namespace state_estimator_plugins

PLUGINLIB_EXPORT_CLASS(
  state_estimator_plugins::ContactDetectionPlugin,
  state_estimator_plugins::PluginBase)
