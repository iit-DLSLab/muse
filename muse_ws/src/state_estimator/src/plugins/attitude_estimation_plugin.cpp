#include "state_estimator/Models/attitude_bias_NLO.hpp"
#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "state_estimator/Models/unitree_low_state_adapter.hpp"
#include "state_estimator/lib.hpp"
#include "state_estimator/plugin.hpp"

#include "iit/commons/geometry/rotations.h"

#include "state_estimator_msgs/msg/attitude.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <exception>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class AttitudeEstimationPlugin : public PluginBase
{
public:
	AttitudeEstimationPlugin() : attitude_(nullptr) {}
	~AttitudeEstimationPlugin() override { if (attitude_ != nullptr) delete attitude_; }

	std::string getName() override { return std::string("AttitudeEstimation"); }
	std::string getDescription() override { return std::string("Attitude Estimation Plugin"); }

	void initialize_() override
	{
		t0 = 0.0;
		xhat_estimated << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		xhat_estimated.head(4).normalize();

		std::string robot_type = "go2";
		std::string imu_topic = "/lowstate";
		std::string pub_topic = "attitude";
		double ki_param = 0.02, kp_param = 10.0;
		std::vector<double> base_R_imu_vec, north_vec, gravity_vec, P_vec, Q_vec, R_vec;

		if (!config_dir_.empty()) {
			try {
				YAML::Node cfg = YAML::LoadFile(config_dir_ + "/attitude_plugin.yaml")["attitude_estimation_plugin"];
				if (cfg["robot_type"]) robot_type = cfg["robot_type"].as<std::string>();
				if (cfg["imu_topic"]) imu_topic = cfg["imu_topic"].as<std::string>();
				if (cfg["pub_topic"]) pub_topic = cfg["pub_topic"].as<std::string>();
				if (cfg["ki"]) ki_param = cfg["ki"].as<double>();
				if (cfg["kp"]) kp_param = cfg["kp"].as<double>();
				if (cfg["base_R_imu"]) base_R_imu_vec = cfg["base_R_imu"].as<std::vector<double>>();
				if (cfg["north_vector"]) north_vec = cfg["north_vector"].as<std::vector<double>>();
				if (cfg["gravity_vector"]) gravity_vec = cfg["gravity_vector"].as<std::vector<double>>();
				if (cfg["P"]) P_vec = cfg["P"].as<std::vector<double>>();
				if (cfg["Q"]) Q_vec = cfg["Q"].as<std::vector<double>>();
				if (cfg["R"]) R_vec = cfg["R"].as<std::vector<double>>();
			} catch (const std::exception& e) {
				RCLCPP_WARN(node_->get_logger(), "Could not load attitude config: %s", e.what());
			}
		}

		ki = ki_param;
		kp = kp_param;
		if (base_R_imu_vec.size() == 9) {
			b_R_imu = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(base_R_imu_vec.data());
		} else {
			b_R_imu = Eigen::Matrix3d::Identity();
		}
		if (north_vec.size() == 3) {
			m_n = Eigen::Map<const Eigen::Vector3d>(north_vec.data());
		} else {
			m_n = Eigen::Vector3d(1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0));
		}
		if (gravity_vec.size() == 3) {
			f_n = Eigen::Map<const Eigen::Vector3d>(gravity_vec.data());
		} else {
			f_n = Eigen::Vector3d(0.0, 0.0, 9.81);
		}

		if (P_vec.size() == 36 && Q_vec.size() == 36 && R_vec.size() == 36) {
			Eigen::Matrix6d P0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(P_vec.data());
			Eigen::Matrix6d Q = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(Q_vec.data());
			Eigen::Matrix6d R = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(R_vec.data());
			attitude_ = new state_estimator::AttitudeBiasXKF(t0, xhat_estimated, P0, Q, R, f_n, m_n, ki, kp);
		} else {
			RCLCPP_WARN(node_->get_logger(), "Missing/invalid P/Q/R - using scaled identity matrices.");
			attitude_ = new state_estimator::AttitudeBiasXKF(
				t0, xhat_estimated,
				1e-6 * Eigen::Matrix6d::Identity(),
				1e-6 * Eigen::Matrix6d::Identity(),
				1e-2 * Eigen::Matrix6d::Identity(),
				f_n, m_n, ki, kp);
		}

		pub_ = node_->create_publisher<state_estimator_msgs::msg::Attitude>(pub_topic, 1);
		createLowStateSubscription(robot_type, imu_topic);
		RCLCPP_INFO(node_->get_logger(),
			"AttitudeEstimationPlugin initialized on topic '%s' for robot_type '%s'",
			imu_topic.c_str(), robot_type.c_str());
	}

	void shutdown_() override {}
	void pause_() override {}
	void resume_() override {}
	void reset_() override {}

private:
	void createLowStateSubscription(const std::string& robot_type, const std::string& imu_topic)
	{
		const auto family = state_estimator::messageFamilyForRobotType(robot_type);
		if (family == state_estimator::UnitreeMessageFamily::Go) {
			go_imu_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
				imu_topic, 250,
				std::bind(&AttitudeEstimationPlugin::callbackGo, this, std::placeholders::_1));
			return;
		}

		hg_imu_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
			imu_topic, 250,
			std::bind(&AttitudeEstimationPlugin::callbackHg, this, std::placeholders::_1));
	}

	void callbackGo(const unitree_go::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"AttitudeEstimationPlugin: failed to adapt unitree_go LowState: %s", error.c_str());
			return;
		}
		processLowState(normalized_low_state_);
	}

	void callbackHg(const unitree_hg::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"AttitudeEstimationPlugin: failed to adapt unitree_hg LowState: %s", error.c_str());
			return;
		}
		processLowState(normalized_low_state_);
	}

	void processLowState(const state_estimator::UnitreeLowState& low_state)
	{
		Eigen::Vector3d omega = low_state.imu.gyroscope;
		Eigen::Vector3d acc = low_state.imu.accelerometer;
		computeAttitude(omega, acc);
	}

	void computeAttitude(Eigen::Vector3d& omega, Eigen::Vector3d& acc)
	{
		if (begin) {
			time_begin_ = node_->now().seconds();
			begin = false;
		}
		time_ = node_->now().seconds() - time_begin_;

		quat_est.w() = xhat_estimated(0);
		quat_est.vec() << xhat_estimated(1), xhat_estimated(2), xhat_estimated(3);
		f_b = b_R_imu * acc;
		m_b = iit::commons::quatToRotMat(quat_est) * m_n;
		z << f_b, m_b;

		attitude_->update(time_, b_R_imu * omega, z);
		xhat_estimated = attitude_->getX();
		xdot = attitude_->calc_f(time_, xhat_estimated, b_R_imu * omega);
		quat_dot.w() = xdot(0);
		quat_dot.vec() << xdot(1), xdot(2), xdot(3);
		omega_filt = iit::commons::quatToOmega(quat_est, quat_dot);

		msg_.header.stamp = node_->now();
		msg_.quaternion[0] = quat_est.w();
		msg_.quaternion[1] = quat_est.x();
		msg_.quaternion[2] = quat_est.y();
		msg_.quaternion[3] = quat_est.z();
		euler_radians = iit::commons::quatToRPY(quat_est);
		euler_degrees = euler_radians * (180.0 / M_PI);
		msg_.roll_deg = euler_degrees(0);
		msg_.pitch_deg = euler_degrees(1);
		msg_.yaw_deg = euler_degrees(2);
		msg_.angular_velocity[0] = omega_filt(0);
		msg_.angular_velocity[1] = omega_filt(1);
		msg_.angular_velocity[2] = omega_filt(2);
		pub_->publish(msg_);
	}

	state_estimator::AttitudeBiasXKF* attitude_;
	state_estimator::UnitreeLowState normalized_low_state_;
	Eigen::Matrix<double,7,1> xhat_estimated;
	Eigen::Matrix<double,3,1> f_n;
	Eigen::Matrix<double,3,1> m_n;
	Eigen::Matrix3d b_R_imu;
	double ki{};
	double kp{};
	double t0{};

	rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr go_imu_sub_;
	rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr hg_imu_sub_;
	rclcpp::Publisher<state_estimator_msgs::msg::Attitude>::SharedPtr pub_;

	Eigen::Quaterniond quat_est;
	Eigen::Vector3d f_b;
	Eigen::Vector3d m_b;
	Eigen::Vector6d z;
	Eigen::Vector7d xdot;
	Eigen::Quaterniond quat_dot;
	Eigen::Vector3d omega_filt;
	Eigen::Vector3d euler_radians;
	Eigen::Vector3d euler_degrees;
	state_estimator_msgs::msg::Attitude msg_;
	double time_{};
	bool begin{true};
	double time_begin_{};
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::AttitudeEstimationPlugin, state_estimator_plugins::PluginBase)
