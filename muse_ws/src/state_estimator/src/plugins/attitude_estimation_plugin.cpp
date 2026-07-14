#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "state_estimator/plugin.hpp"

#include <iit/commons/geometry/rotations.h>
#include <sensor_msgs/msg/imu.hpp>
#include <state_estimator_msgs/msg/attitude.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class AttitudeEstimationPlugin : public PluginBase
{
public:
  std::string getName() const override { return "AttitudeEstimation"; }
  std::string getDescription() const override { return "Attitude Estimation Plugin"; }

private:
  void initialize_() override
  {
    const auto imu_topic = parameter<std::string>(
      "attitude_estimation_plugin.imu_topic", "/imu");
    const auto pub_topic = parameter<std::string>(
      "attitude_estimation_plugin.pub_topic", "~/attitude");
    ki_ = parameter<double>("attitude_estimation_plugin.ki", 0.02);
    kp_ = parameter<double>("attitude_estimation_plugin.kp", 10.0);

    const auto base_R_imu = parameter<std::vector<double>>(
      "attitude_estimation_plugin.base_R_imu", {});
    const auto north = parameter<std::vector<double>>(
      "attitude_estimation_plugin.north_vector", {});
    const auto gravity = parameter<std::vector<double>>(
      "attitude_estimation_plugin.gravity_vector", {});
    const auto P = parameter<std::vector<double>>("attitude_estimation_plugin.P", {});
    const auto Q = parameter<std::vector<double>>("attitude_estimation_plugin.Q", {});
    const auto R = parameter<std::vector<double>>("attitude_estimation_plugin.R", {});

    if (base_R_imu.size() == 9) {
      base_R_imu_ = Eigen::Map<const Eigen::Matrix3d>(base_R_imu.data());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid base_R_imu; using identity");
      base_R_imu_.setIdentity();
    }
    if (north.size() == 3) {
      north_ = Eigen::Map<const Eigen::Vector3d>(north.data());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid north_vector; using default");
      north_ << 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0);
    }
    if (gravity.size() == 3) {
      gravity_ = Eigen::Map<const Eigen::Vector3d>(gravity.data());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid gravity_vector; using default");
      gravity_ << 0.0, 0.0, 9.81;
    }
    if (P.size() != 36 || Q.size() != 36 || R.size() != 36) {
      throw std::runtime_error("attitude P, Q, and R parameters must each contain 36 values");
    }
    P_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P.data());
    Q_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q.data());
    R_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(R.data());
    resetFilter();

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr imu) { imuCallback(imu); });
    pub_ = node_->create_publisher<state_estimator_msgs::msg::Attitude>(pub_topic, 1);
    RCLCPP_INFO(
      node_->get_logger(), "AttitudeEstimationPlugin reading %s", imu_topic.c_str());
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
  {
    if (isPaused()) return;

    Eigen::Vector3d omega(
      imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    Eigen::Vector3d acceleration(
      imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    computeAttitude(omega, acceleration);
  }

  void computeAttitude(Eigen::Vector3d& omega, Eigen::Vector3d& acceleration)
  {
    const double now = node_->now().seconds();
    if (begin_) {
      time_begin_ = now;
      begin_ = false;
    }
    const double time = now - time_begin_;

    Eigen::Quaterniond quaternion;
    quaternion.w() = state_(0);
    quaternion.vec() << state_(1), state_(2), state_(3);
    const Eigen::Vector3d force_base = base_R_imu_ * acceleration;
    const Eigen::Vector3d magnetic_base = iit::commons::quatToRotMat(quaternion) * north_;
    Eigen::Vector6d measurement;
    measurement << force_base, magnetic_base;

    attitude_->update(time, base_R_imu_ * omega, measurement);
    state_ = attitude_->getX();

    const Eigen::Vector7d derivative =
      attitude_->calc_f(time, state_, base_R_imu_ * omega);
    Eigen::Quaterniond quaternion_dot;
    quaternion_dot.w() = derivative(0);
    quaternion_dot.vec() << derivative(1), derivative(2), derivative(3);
    const Eigen::Vector3d filtered_omega =
      iit::commons::quatToOmega(quaternion, quaternion_dot);
    const Eigen::Vector3d euler_degrees =
      iit::commons::quatToRPY(quaternion) * (180.0 / M_PI);

    state_estimator_msgs::msg::Attitude message;
    message.header.stamp = node_->now();
    message.quaternion = {{quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()}};
    message.roll_deg = euler_degrees(0);
    message.pitch_deg = euler_degrees(1);
    message.yaw_deg = euler_degrees(2);
    message.angular_velocity = {{filtered_omega(0), filtered_omega(1), filtered_omega(2)}};
    pub_->publish(message);
  }

  void resetFilter()
  {
    state_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    attitude_ = std::make_unique<state_estimator::AttitudeBiasXKF>(
      0.0, state_, P_, Q_, R_, gravity_, north_, ki_, kp_);
    begin_ = true;
    time_begin_ = 0.0;
  }

  void shutdown_() override
  {
    imu_sub_.reset();
    pub_.reset();
  }
  void pause_() override {}
  void resume_() override {}
  void reset_() override { resetFilter(); }

  std::unique_ptr<state_estimator::AttitudeBiasXKF> attitude_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<state_estimator_msgs::msg::Attitude>::SharedPtr pub_;

  Eigen::Vector7d state_;
  Eigen::Matrix6d P_;
  Eigen::Matrix6d Q_;
  Eigen::Matrix6d R_;
  Eigen::Vector3d gravity_;
  Eigen::Vector3d north_;
  Eigen::Matrix3d base_R_imu_;
  double ki_{0.02};
  double kp_{10.0};
  bool begin_{true};
  double time_begin_{0.0};
};

}  // namespace state_estimator_plugins

PLUGINLIB_EXPORT_CLASS(
  state_estimator_plugins::AttitudeEstimationPlugin,
  state_estimator_plugins::PluginBase)
