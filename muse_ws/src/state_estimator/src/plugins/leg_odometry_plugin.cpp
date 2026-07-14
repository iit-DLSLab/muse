#include "state_estimator/Models/joint_state_snapshot.hpp"
#include "state_estimator/Models/leg_odometry.hpp"
#include "state_estimator/plugin.hpp"

#include <iit/commons/geometry/rotations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <state_estimator_msgs/msg/attitude.hpp>
#include <state_estimator_msgs/msg/contact_detection.hpp>
#include <state_estimator_msgs/msg/joint_state_with_acceleration.hpp>
#include <state_estimator_msgs/msg/leg_odometry.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class LegOdometryPlugin : public PluginBase
{
public:
  std::string getName() const override { return "LegOdometry"; }
  std::string getDescription() const override { return "Pinocchio leg odometry plugin"; }

private:
  void initialize_() override
  {
    const auto urdf_path = resolveResource(parameter<std::string>(
      "leg_odometry_plugin.urdf_path", "urdfs/anymal.urdf"));
    auto foot_frames = parameter<std::vector<std::string>>(
      "leg_odometry_plugin.foot_frame_names", defaultFootFrameNames());
    auto joint_names = parameter<std::vector<std::string>>(
      "leg_odometry_plugin.joint_names", defaultJointNames());
    const auto base_frame = parameter<std::string>(
      "leg_odometry_plugin.base_frame_name", "base");
    const auto rotation = parameter<std::vector<double>>(
      "leg_odometry_plugin.base_R_imu", defaultBaseRImuVector());

    if (rotation.size() == 9) {
      base_R_imu_ = Eigen::Map<
        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
    } else {
      RCLCPP_WARN(node_->get_logger(), "leg_odometry base_R_imu invalid; using default");
      const auto fallback = defaultBaseRImuVector();
      base_R_imu_ = Eigen::Map<
        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(fallback.data());
    }
    if (foot_frames.size() != 4) {
      RCLCPP_WARN(node_->get_logger(), "leg_odometry needs four foot frames; using defaults");
      foot_frames = defaultFootFrameNames();
    }
    if (joint_names.size() != 12) {
      RCLCPP_WARN(node_->get_logger(), "leg_odometry needs twelve joint names; using defaults");
      joint_names = defaultJointNames();
    }
    if (!model_.configure(urdf_path, foot_frames, joint_names, base_frame)) {
      throw std::runtime_error("failed to configure leg odometry: " + model_.lastError());
    }

    const auto imu_topic = parameter<std::string>("leg_odometry_plugin.imu_topic", "/imu");
    const auto actuator_topic = parameter<std::string>(
      "leg_odometry_plugin.actuator_state_topic", "/actuator_state");
    const auto contact_topic = parameter<std::string>(
      "leg_odometry_plugin.contact_topic", "~/contact_detection");
    const auto attitude_topic = parameter<std::string>(
      "leg_odometry_plugin.attitude_topic", "~/attitude");
    const auto pub_topic = parameter<std::string>(
      "leg_odometry_plugin.pub_topic", "~/leg_odometry");

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        if (!isPaused()) { latest_imu_ = *message; has_imu_ = true; }
      });
    actuator_sub_ = node_->create_subscription<
      state_estimator_msgs::msg::JointStateWithAcceleration>(
      actuator_topic, rclcpp::SensorDataQoS(),
      [this](state_estimator_msgs::msg::JointStateWithAcceleration::ConstSharedPtr message) {
        if (isPaused()) return;
        latest_joints_.name = message->name;
        latest_joints_.position = message->position;
        latest_joints_.velocity = message->velocity;
        latest_joints_.acceleration = message->acceleration;
        latest_joints_.effort = message->effort;
        latest_joint_stamp_ = message->header.stamp;
        has_joints_ = true;
      });
    contact_sub_ = node_->create_subscription<state_estimator_msgs::msg::ContactDetection>(
      contact_topic, rclcpp::SensorDataQoS(),
      [this](state_estimator_msgs::msg::ContactDetection::ConstSharedPtr message) {
        if (isPaused()) return;
        latest_contact_ = *message;
        has_contact_ = true;
        publishEstimate();
      });
    attitude_sub_ = node_->create_subscription<state_estimator_msgs::msg::Attitude>(
      attitude_topic, rclcpp::SensorDataQoS(),
      [this](state_estimator_msgs::msg::Attitude::ConstSharedPtr message) {
        if (!isPaused()) { latest_attitude_ = *message; has_attitude_ = true; }
      });
    pub_ = node_->create_publisher<state_estimator_msgs::msg::LegOdometry>(
      pub_topic, rclcpp::QoS(250));

    RCLCPP_INFO(
      node_->get_logger(), "LegOdometryPlugin reading actuator state from %s",
      actuator_topic.c_str());
  }

  void publishEstimate()
  {
    if (!has_imu_ || !has_joints_ || !has_contact_ || !has_attitude_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "LegOdometryPlugin waiting for inputs: imu=%d actuator=%d contact=%d attitude=%d",
        has_imu_, has_joints_, has_contact_, has_attitude_);
      return;
    }

    const Eigen::Vector3d omega_imu(
      latest_imu_.angular_velocity.x,
      latest_imu_.angular_velocity.y,
      latest_imu_.angular_velocity.z);
    const Eigen::Vector3d omega_base = base_R_imu_ * omega_imu;

    Eigen::Quaterniond quaternion;
    quaternion.w() = latest_attitude_.quaternion[0];
    quaternion.vec() << latest_attitude_.quaternion[1],
      latest_attitude_.quaternion[2], latest_attitude_.quaternion[3];
    if (quaternion.norm() > 1e-9) quaternion.normalize();
    else quaternion.setIdentity();
    const Eigen::Matrix3d world_R_base =
      iit::commons::quatToRotMat(quaternion).transpose();

    const std::array<bool, 4> stance{{
      latest_contact_.stance_lf,
      latest_contact_.stance_rf,
      latest_contact_.stance_lh,
      latest_contact_.stance_rh
    }};
    state_estimator::LegOdometryEstimate estimate;
    if (!model_.update(latest_joints_, omega_base, stance, world_R_base, estimate)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Leg odometry update failed: %s", model_.lastError().c_str());
      return;
    }

    state_estimator_msgs::msg::LegOdometry output;
    output.header.stamp = outputStamp();
    for (std::size_t axis = 0; axis < 3; ++axis) {
      output.lin_vel_lf[axis] = estimate.base_velocity_from_foot_base[0][axis];
      output.lin_vel_rf[axis] = estimate.base_velocity_from_foot_base[1][axis];
      output.lin_vel_lh[axis] = estimate.base_velocity_from_foot_base[2][axis];
      output.lin_vel_rh[axis] = estimate.base_velocity_from_foot_base[3][axis];
      output.base_velocity[axis] = estimate.base_velocity_world[axis];
    }
    output.stance_count = static_cast<uint8_t>(estimate.stance_count);
    output.base_height_valid = estimate.base_height_valid;
    output.base_height = estimate.base_height;
    for (std::size_t leg = 0; leg < stance.size(); ++leg) {
      output.stance[leg] = stance[leg];
      output.base_to_foot_position_world_z[leg] =
        (world_R_base * estimate.kinematics.foot_position_base[leg]).z();
    }
    pub_->publish(output);
  }

  builtin_interfaces::msg::Time outputStamp() const
  {
    if (rclcpp::Time(latest_contact_.header.stamp).nanoseconds() != 0) {
      return latest_contact_.header.stamp;
    }
    if (rclcpp::Time(latest_joint_stamp_).nanoseconds() != 0) return latest_joint_stamp_;
    if (rclcpp::Time(latest_imu_.header.stamp).nanoseconds() != 0) return latest_imu_.header.stamp;
    return static_cast<builtin_interfaces::msg::Time>(node_->now());
  }

  static std::string resolveResource(const std::string& path)
  {
    const std::filesystem::path requested(path);
    if (requested.is_absolute()) return requested.string();
    return (std::filesystem::path(
      ament_index_cpp::get_package_share_directory("state_estimator")) / requested).string();
  }

  static std::vector<std::string> defaultFootFrameNames()
  {
    return {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  }

  static std::vector<std::string> defaultJointNames()
  {
    return {
      "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
      "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  }

  static std::vector<double> defaultBaseRImuVector()
  {
    return {-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0};
  }

  void clearInputs()
  {
    has_imu_ = false;
    has_joints_ = false;
    has_contact_ = false;
    has_attitude_ = false;
  }

  void shutdown_() override
  {
    imu_sub_.reset();
    actuator_sub_.reset();
    contact_sub_.reset();
    attitude_sub_.reset();
    pub_.reset();
  }
  void pause_() override {}
  void resume_() override {}
  void reset_() override { clearInputs(); }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<
    state_estimator_msgs::msg::JointStateWithAcceleration>::SharedPtr actuator_sub_;
  rclcpp::Subscription<state_estimator_msgs::msg::ContactDetection>::SharedPtr contact_sub_;
  rclcpp::Subscription<state_estimator_msgs::msg::Attitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<state_estimator_msgs::msg::LegOdometry>::SharedPtr pub_;

  sensor_msgs::msg::Imu latest_imu_;
  state_estimator::JointStateSnapshot latest_joints_;
  builtin_interfaces::msg::Time latest_joint_stamp_;
  state_estimator_msgs::msg::ContactDetection latest_contact_;
  state_estimator_msgs::msg::Attitude latest_attitude_;
  state_estimator::LegOdometryModel model_;
  Eigen::Matrix3d base_R_imu_{Eigen::Matrix3d::Identity()};
  bool has_imu_{false};
  bool has_joints_{false};
  bool has_contact_{false};
  bool has_attitude_{false};
};

}  // namespace state_estimator_plugins

PLUGINLIB_EXPORT_CLASS(
  state_estimator_plugins::LegOdometryPlugin,
  state_estimator_plugins::PluginBase)
