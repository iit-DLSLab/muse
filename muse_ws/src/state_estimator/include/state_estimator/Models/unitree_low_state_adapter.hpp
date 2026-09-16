#ifndef STATE_ESTIMATOR_MODELS_UNITREE_LOW_STATE_ADAPTER_HPP
#define STATE_ESTIMATOR_MODELS_UNITREE_LOW_STATE_ADAPTER_HPP

#include "unitree_go/msg/low_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <string>
#include <vector>

namespace state_estimator
{

enum class UnitreeMessageFamily
{
	Go,
	Hg
};

struct UnitreeMotorState
{
	double q{0.0};
	double dq{0.0};
	double ddq{0.0};
	double tau_est{0.0};
};

struct UnitreeImuState
{
	Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
	Eigen::Vector3d gyroscope{Eigen::Vector3d::Zero()};
	Eigen::Vector3d accelerometer{Eigen::Vector3d::Zero()};
	Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
};

struct UnitreeLowState
{
	UnitreeImuState imu;
	std::vector<UnitreeMotorState> motors;
};

inline std::string normalizeRobotType(std::string robot_type)
{
	std::transform(robot_type.begin(), robot_type.end(), robot_type.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	return robot_type;
}

inline UnitreeMessageFamily messageFamilyForRobotType(const std::string& robot_type)
{
	const std::string normalized = normalizeRobotType(robot_type);
	if (normalized == "a2" || normalized == "b2") {
		return UnitreeMessageFamily::Hg;
	}
	return UnitreeMessageFamily::Go;
}

inline std::vector<std::size_t> defaultMotorIndices(std::size_t count)
{
	std::vector<std::size_t> indices(count, 0);
	std::iota(indices.begin(), indices.end(), 0);
	return indices;
}

inline bool isFinite(const Eigen::Vector3d& value)
{
	return value.allFinite();
}

inline bool isFinite(const Eigen::Quaterniond& value)
{
	return std::isfinite(value.w()) &&
		   std::isfinite(value.x()) &&
		   std::isfinite(value.y()) &&
		   std::isfinite(value.z());
}

template <typename RosLowState>
bool fromRosLowState(const RosLowState& msg, UnitreeLowState& low_state, std::string* error = nullptr)
{
	Eigen::Quaterniond orientation(
		static_cast<double>(msg.imu_state.quaternion[0]),
		static_cast<double>(msg.imu_state.quaternion[1]),
		static_cast<double>(msg.imu_state.quaternion[2]),
		static_cast<double>(msg.imu_state.quaternion[3]));
	Eigen::Vector3d gyroscope(
		static_cast<double>(msg.imu_state.gyroscope[0]),
		static_cast<double>(msg.imu_state.gyroscope[1]),
		static_cast<double>(msg.imu_state.gyroscope[2]));
	Eigen::Vector3d accelerometer(
		static_cast<double>(msg.imu_state.accelerometer[0]),
		static_cast<double>(msg.imu_state.accelerometer[1]),
		static_cast<double>(msg.imu_state.accelerometer[2]));
	Eigen::Vector3d rpy(
		static_cast<double>(msg.imu_state.rpy[0]),
		static_cast<double>(msg.imu_state.rpy[1]),
		static_cast<double>(msg.imu_state.rpy[2]));

	if (!isFinite(orientation) || orientation.norm() < 1e-9 ||
		!isFinite(gyroscope) || !isFinite(accelerometer) || !isFinite(rpy)) {
		if (error) {
			*error = "non-finite IMU field in Unitree LowState";
		}
		return false;
	}

	orientation.normalize();
	low_state.imu.orientation = orientation;
	low_state.imu.gyroscope = gyroscope;
	low_state.imu.accelerometer = accelerometer;
	low_state.imu.rpy = rpy;

	low_state.motors.resize(msg.motor_state.size());
	for (std::size_t i = 0; i < msg.motor_state.size(); ++i) {
		const auto& motor = msg.motor_state[i];
		UnitreeMotorState normalized_motor;
		normalized_motor.q = static_cast<double>(motor.q);
		normalized_motor.dq = static_cast<double>(motor.dq);
		normalized_motor.ddq = static_cast<double>(motor.ddq);
		normalized_motor.tau_est = static_cast<double>(motor.tau_est);

		if (!std::isfinite(normalized_motor.q) ||
			!std::isfinite(normalized_motor.dq) ||
			!std::isfinite(normalized_motor.ddq) ||
			!std::isfinite(normalized_motor.tau_est)) {
			if (error) {
				*error = "non-finite motor field in Unitree LowState";
			}
			return false;
		}

		low_state.motors[i] = normalized_motor;
	}

	if (error) {
		error->clear();
	}
	return true;
}

inline bool fromRos(
	const unitree_go::msg::LowState& msg,
	UnitreeLowState& low_state,
	std::string* error = nullptr)
{
	return fromRosLowState(msg, low_state, error);
}

inline bool fromRos(
	const unitree_hg::msg::LowState& msg,
	UnitreeLowState& low_state,
	std::string* error = nullptr)
{
	return fromRosLowState(msg, low_state, error);
}

} // namespace state_estimator

#endif // STATE_ESTIMATOR_MODELS_UNITREE_LOW_STATE_ADAPTER_HPP
