#include "state_estimator/Models/sensor_fusion.hpp"
#include "state_estimator/plugin.hpp"

#include <iit/commons/geometry/rotations.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <state_estimator_msgs/msg/attitude.hpp>
#include <state_estimator_msgs/msg/leg_odometry.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class SensorFusionPlugin : public PluginBase
{
public:
  std::string getName() const override { return "SensorFusion"; }
  std::string getDescription() const override { return "Proprioceptive sensor fusion plugin"; }

private:
  void initialize_() override
  {
    const auto P_values = parameter<std::vector<double>>("sensor_fusion_plugin.P", {});
    const auto Q_values = parameter<std::vector<double>>("sensor_fusion_plugin.Q", {});
    const auto R_values = parameter<std::vector<double>>("sensor_fusion_plugin.R", {});
    if (P_values.size() != 36 || Q_values.size() != 36 || R_values.size() != 9) {
      throw std::runtime_error(
        "sensor fusion P and Q must contain 36 values and R must contain 9 values");
    }
    P_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P_values.data());
    Q_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q_values.data());
    R_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_values.data());

    const auto rotation = parameter<std::vector<double>>(
      "attitude_estimation_plugin.base_R_imu", std::vector<double>(9, 0.0));
    if (rotation.size() == 9) {
      base_R_imu_ = Eigen::Map<
        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
    } else {
      RCLCPP_WARN(node_->get_logger(), "base_R_imu invalid; using identity");
      base_R_imu_.setIdentity();
    }

    frame_id_ = parameter<std::string>("sensor_fusion_plugin.frame_id", "world");
    child_frame_id_ = parameter<std::string>("sensor_fusion_plugin.child_frame_id", "base");
    z_height_correction_enabled_ = parameter<bool>(
      "sensor_fusion_plugin.z_height_correction.enabled", true);
    z_height_assume_flat_ground_ = parameter<bool>(
      "sensor_fusion_plugin.z_height_correction.assume_flat_ground", false);
    z_height_ground_height_ = parameter<double>(
      "sensor_fusion_plugin.z_height_correction.ground_height", 0.0);
    z_height_measurement_variance_ = parameter<double>(
      "sensor_fusion_plugin.z_height_correction.measurement_variance", 1.0e-8);
    z_height_max_innovation_ = parameter<double>(
      "sensor_fusion_plugin.z_height_correction.max_innovation", 0.25);
    z_height_filter_alpha_ = parameter<double>(
      "sensor_fusion_plugin.z_height_correction.filter_alpha", 1.0);
    z_height_min_stance_legs_ = std::clamp(
      static_cast<int>(parameter<int64_t>(
        "sensor_fusion_plugin.z_height_correction.min_stance_legs", 1)), 1, 4);

    const auto imu_topic = parameter<std::string>("sensor_fusion_plugin.imu_topic", "/imu");
    const auto attitude_topic = parameter<std::string>(
      "sensor_fusion_plugin.attitude_topic", "~/attitude");
    const auto leg_odometry_topic = parameter<std::string>(
      "sensor_fusion_plugin.leg_odometry_topic", "~/leg_odometry");
    const auto pub_topic = parameter<std::string>(
      "sensor_fusion_plugin.pub_topic", "~/sensor_fusion");

    resetFilter();
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        if (!isPaused()) { latest_imu_ = *message; has_imu_ = true; }
      });
    attitude_sub_ = node_->create_subscription<state_estimator_msgs::msg::Attitude>(
      attitude_topic, rclcpp::SensorDataQoS(),
      [this](state_estimator_msgs::msg::Attitude::ConstSharedPtr message) {
        if (!isPaused()) { latest_attitude_ = *message; has_attitude_ = true; }
      });
    leg_odometry_sub_ = node_->create_subscription<state_estimator_msgs::msg::LegOdometry>(
      leg_odometry_topic, rclcpp::SensorDataQoS(),
      [this](state_estimator_msgs::msg::LegOdometry::ConstSharedPtr message) {
        if (isPaused()) return;
        latest_leg_odometry_ = *message;
        has_leg_odometry_ = true;
        update();
      });
    pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(pub_topic, rclcpp::QoS(250));

    RCLCPP_INFO(
      node_->get_logger(), "SensorFusionPlugin reading %s, %s, and %s",
      imu_topic.c_str(), attitude_topic.c_str(), leg_odometry_topic.c_str());
  }

  void update()
  {
    if (!has_imu_ || !has_attitude_ || !has_leg_odometry_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "SensorFusionPlugin waiting for inputs: imu=%d attitude=%d leg_odometry=%d",
        has_imu_, has_attitude_, has_leg_odometry_);
      return;
    }

    const Eigen::Vector3d acceleration(
      latest_imu_.linear_acceleration.x,
      latest_imu_.linear_acceleration.y,
      latest_imu_.linear_acceleration.z);
    Eigen::Vector3d omega;
    omega << latest_attitude_.angular_velocity[0],
      latest_attitude_.angular_velocity[1], latest_attitude_.angular_velocity[2];
    Eigen::Quaterniond quaternion;
    quaternion.w() = latest_attitude_.quaternion[0];
    quaternion.vec() << latest_attitude_.quaternion[1],
      latest_attitude_.quaternion[2], latest_attitude_.quaternion[3];
    const Eigen::Matrix3d world_R_base =
      iit::commons::quatToRotMat(quaternion).transpose();
    Eigen::Vector3d base_velocity;
    base_velocity << latest_leg_odometry_.base_velocity[0],
      latest_leg_odometry_.base_velocity[1], latest_leg_odometry_.base_velocity[2];

    compute(acceleration, world_R_base, base_velocity, omega, quaternion, outputStamp());
  }

  void compute(
    const Eigen::Vector3d& acceleration,
    const Eigen::Matrix3d& world_R_base,
    const Eigen::Vector3d& base_velocity,
    const Eigen::Vector3d& omega,
    const Eigen::Quaterniond& quaternion,
    const rclcpp::Time& stamp)
  {
    if (begin_) {
      time_begin_ = stamp.seconds();
      last_update_time_ = 0.0;
      begin_ = false;
    }
    const double time = stamp.seconds() - time_begin_;
    if (time < last_update_time_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "SensorFusionPlugin received out-of-order timestamps; skipping update");
      return;
    }

    const Eigen::Vector3d force_base = base_R_imu_ * acceleration;
    const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    const Eigen::Vector3d input = world_R_base * force_base + gravity;
    filter_->predict(time, input);
    filter_->update(time, base_velocity);
    applyZHeightCorrection(time);
    const Eigen::Matrix<double, 6, 1> state = filter_->getX();
    last_update_time_ = time;

    nav_msgs::msg::Odometry output;
    output.header.stamp = stamp;
    output.header.frame_id = frame_id_;
    output.child_frame_id = child_frame_id_;
    output.pose.pose.orientation.w = quaternion.w();
    output.pose.pose.orientation.x = quaternion.x();
    output.pose.pose.orientation.y = quaternion.y();
    output.pose.pose.orientation.z = quaternion.z();
    output.pose.pose.position.x = state(0);
    output.pose.pose.position.y = state(1);
    output.pose.pose.position.z = state(2);
    output.twist.twist.linear.x = state(3);
    output.twist.twist.linear.y = state(4);
    output.twist.twist.linear.z = state(5);
    output.twist.twist.angular.x = omega(0);
    output.twist.twist.angular.y = omega(1);
    output.twist.twist.angular.z = omega(2);
    pub_->publish(output);
  }

  void applyZHeightCorrection(double time)
  {
    if (!z_height_correction_enabled_) return;
    if (z_height_assume_flat_ground_) {
      if (!latest_leg_odometry_.base_height_valid ||
          latest_leg_odometry_.stance_count < z_height_min_stance_legs_) return;
      const double measurement =
        latest_leg_odometry_.base_height + z_height_ground_height_;
      if (std::isfinite(measurement)) applyBoundedZMeasurement(time, measurement);
      return;
    }

    const double current_base_z = filter_->getX()(2);
    double measurement_sum = 0.0;
    std::size_t valid_measurements = 0;
    for (std::size_t leg = 0; leg < foot_anchor_world_z_.size(); ++leg) {
      const bool stance = latest_leg_odometry_.stance[leg];
      const double base_to_foot_z =
        latest_leg_odometry_.base_to_foot_position_world_z[leg];
      if (!stance) {
        foot_anchor_valid_[leg] = false;
        previous_foot_stance_[leg] = false;
        continue;
      }
      if (!std::isfinite(base_to_foot_z)) continue;
      if (!previous_foot_stance_[leg] || !foot_anchor_valid_[leg]) {
        foot_anchor_world_z_[leg] = current_base_z + base_to_foot_z;
        foot_anchor_valid_[leg] = true;
      }
      measurement_sum += foot_anchor_world_z_[leg] - base_to_foot_z;
      ++valid_measurements;
      previous_foot_stance_[leg] = true;
    }
    if (valid_measurements >= static_cast<std::size_t>(z_height_min_stance_legs_)) {
      applyBoundedZMeasurement(time, measurement_sum / valid_measurements);
    }
  }

  void applyBoundedZMeasurement(double time, double measurement)
  {
    if (z_height_measurement_variance_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Z-height correction disabled by non-positive measurement variance");
      return;
    }
    const double alpha = std::clamp(z_height_filter_alpha_, 0.0, 1.0);
    if (!has_filtered_z_height_) {
      filtered_z_height_ = measurement;
      has_filtered_z_height_ = true;
    } else {
      filtered_z_height_ = alpha * measurement + (1.0 - alpha) * filtered_z_height_;
    }

    const double innovation = filtered_z_height_ - filter_->getX()(2);
    if (z_height_max_innovation_ > 0.0 &&
        std::abs(innovation) > z_height_max_innovation_) {
      filtered_z_height_ = filter_->getX()(2) +
        std::copysign(z_height_max_innovation_, innovation);
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Clamped z-height innovation %.3f to %.3f",
        innovation, z_height_max_innovation_);
    }
    filter_->updateZPosition(time, filtered_z_height_, z_height_measurement_variance_);
  }

  rclcpp::Time outputStamp() const
  {
    if (rclcpp::Time(latest_leg_odometry_.header.stamp).nanoseconds() != 0) {
      return rclcpp::Time(latest_leg_odometry_.header.stamp);
    }
    if (rclcpp::Time(latest_attitude_.header.stamp).nanoseconds() != 0) {
      return rclcpp::Time(latest_attitude_.header.stamp);
    }
    if (rclcpp::Time(latest_imu_.header.stamp).nanoseconds() != 0) {
      return rclcpp::Time(latest_imu_.header.stamp);
    }
    return node_->now();
  }

  void resetFilter()
  {
    Eigen::Matrix<double, 6, 1> initial_state;
    initial_state << 0.0, 0.0, 0.45, 0.0, 0.0, 0.0;
    filter_ = std::make_unique<state_estimator::KFSensorFusion>(
      0.0, initial_state, P_, Q_, R_, false, false);
    has_imu_ = false;
    has_attitude_ = false;
    has_leg_odometry_ = false;
    begin_ = true;
    time_begin_ = 0.0;
    last_update_time_ = 0.0;
    has_filtered_z_height_ = false;
    foot_anchor_valid_.fill(false);
    previous_foot_stance_.fill(false);
  }

  void shutdown_() override
  {
    imu_sub_.reset();
    attitude_sub_.reset();
    leg_odometry_sub_.reset();
    pub_.reset();
  }
  void pause_() override {}
  void resume_() override {}
  void reset_() override { resetFilter(); }

  std::unique_ptr<state_estimator::KFSensorFusion> filter_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<state_estimator_msgs::msg::Attitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<state_estimator_msgs::msg::LegOdometry>::SharedPtr leg_odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  sensor_msgs::msg::Imu latest_imu_;
  state_estimator_msgs::msg::Attitude latest_attitude_;
  state_estimator_msgs::msg::LegOdometry latest_leg_odometry_;
  Eigen::Matrix<double, 6, 6> P_;
  Eigen::Matrix<double, 6, 6> Q_;
  Eigen::Matrix<double, 3, 3> R_;
  Eigen::Matrix3d base_R_imu_{Eigen::Matrix3d::Identity()};
  std::string frame_id_;
  std::string child_frame_id_;
  bool has_imu_{false};
  bool has_attitude_{false};
  bool has_leg_odometry_{false};
  bool begin_{true};
  double time_begin_{0.0};
  double last_update_time_{0.0};
  bool z_height_correction_enabled_{true};
  bool z_height_assume_flat_ground_{false};
  int z_height_min_stance_legs_{1};
  double z_height_ground_height_{0.0};
  double z_height_measurement_variance_{1.0e-8};
  double z_height_max_innovation_{0.25};
  double z_height_filter_alpha_{1.0};
  bool has_filtered_z_height_{false};
  double filtered_z_height_{0.0};
  std::array<bool, 4> foot_anchor_valid_{{false, false, false, false}};
  std::array<bool, 4> previous_foot_stance_{{false, false, false, false}};
  std::array<double, 4> foot_anchor_world_z_{{0.0, 0.0, 0.0, 0.0}};
};

}  // namespace state_estimator_plugins

PLUGINLIB_EXPORT_CLASS(
  state_estimator_plugins::SensorFusionPlugin,
  state_estimator_plugins::PluginBase)
