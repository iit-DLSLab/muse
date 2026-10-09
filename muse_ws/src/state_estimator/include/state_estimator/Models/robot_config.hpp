#ifndef STATE_ESTIMATOR_MODELS_ROBOT_CONFIG_HPP
#define STATE_ESTIMATOR_MODELS_ROBOT_CONFIG_HPP

#include "state_estimator/Models/unitree_low_state_adapter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace state_estimator
{

struct ContactConfig
{
	double threshold{30.0};
	double threshold_low{30.0};
	double threshold_high{30.0};
	bool use_hysteresis{false};
	Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
	bool use_force_norm{false};
	bool use_absolute_normal_force{false};
};

struct GrfConfig
{
	double force_sign{-1.0};
	bool use_inverse_dynamics_compensation{true};
	bool use_joint_acceleration{false};
	double input_filter_alpha{0.2};
	double force_filter_alpha{0.2};
	double solve_damping{1e-4};
	double min_singular_value{1e-6};
	double max_condition_number{1e4};
};

struct RobotConfig
{
	std::string robot_type{"go2"};
	std::string low_state_topic{"/lowstate"};
	std::string urdf_path;
	std::vector<std::string> motor_joint_names;
	std::vector<std::size_t> motor_indices;
	std::vector<std::string> feet_frame_names;
	ContactConfig contact;
	GrfConfig grf;
};

inline std::vector<std::string> defaultQuadrupedMotorJointNames()
{
	return {
		"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
		"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
		"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
		"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
	};
}

inline std::vector<std::string> defaultQuadrupedFeetFrameNames()
{
	return {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
}

inline bool usesRawFootForce(const std::string& robot_type)
{
	return normalizeRobotType(robot_type) == "go2";
}

inline std::vector<std::size_t> sizeTVectorFromYaml(const YAML::Node& node)
{
	std::vector<std::size_t> converted;
	for (const int value : node.as<std::vector<int>>()) {
		if (value >= 0) {
			converted.push_back(static_cast<std::size_t>(value));
		}
	}
	return converted;
}

inline Eigen::Vector3d normalizedVector3FromYaml(
	const YAML::Node& node,
	const Eigen::Vector3d& fallback)
{
	const auto values = node.as<std::vector<double>>();
	if (values.size() != 3) {
		return fallback;
	}
	const Eigen::Vector3d vector(values[0], values[1], values[2]);
	return (!vector.allFinite() || vector.norm() < 1e-9) ? fallback : vector.normalized();
}

inline std::string resolvePackagePath(const std::string& input)
{
	std::string resolved = input;
	const std::string find_token = "$(find ";
	const std::size_t token_pos = resolved.find(find_token);
	if (token_pos == std::string::npos) {
		return resolved;
	}

	const std::size_t package_start = token_pos + find_token.length();
	const std::size_t package_end = resolved.find(")", package_start);
	if (package_end == std::string::npos) {
		return resolved;
	}

	const std::string package_name = resolved.substr(package_start, package_end - package_start);
	const std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
	resolved.replace(token_pos, package_end - token_pos + 1, package_path);
	return resolved;
}

inline bool loadRobotConfig(
	const std::string& config_dir,
	RobotConfig& config,
	std::string& error)
{
	config = RobotConfig{};
	config.motor_joint_names = defaultQuadrupedMotorJointNames();
	config.feet_frame_names = defaultQuadrupedFeetFrameNames();

	if (config_dir.empty()) {
		error = "config_dir is empty; cannot load robot.yaml";
		return false;
	}

	try {
		const YAML::Node root = YAML::LoadFile(config_dir + "/robot.yaml")["robot"];
		if (!root || !root["type"]) {
			error = "robot.yaml must contain robot.type";
			return false;
		}

		config.robot_type = normalizeRobotType(root["type"].as<std::string>());
		if (root["low_state_topic"]) {
			config.low_state_topic = root["low_state_topic"].as<std::string>();
		}

		if (root["urdf_path"]) {
			config.urdf_path = resolvePackagePath(root["urdf_path"].as<std::string>());
		} else {
			config.urdf_path = (
				std::filesystem::path(config_dir).parent_path() / "urdfs" /
				(config.robot_type + ".urdf")).string();
		}

		if (root["motor_joint_names"]) {
			config.motor_joint_names = root["motor_joint_names"].as<std::vector<std::string>>();
		}
		if (root["motor_indices"]) {
			config.motor_indices = sizeTVectorFromYaml(root["motor_indices"]);
		} else {
			config.motor_indices = defaultMotorIndicesForRobotType(
				config.robot_type, config.motor_joint_names.size());
		}
		if (root["feet_frame_names"]) {
			config.feet_frame_names = root["feet_frame_names"].as<std::vector<std::string>>();
		}

		const YAML::Node contact = root["contact"];
		if (contact) {
			if (contact["threshold"]) config.contact.threshold = contact["threshold"].as<double>();
			config.contact.threshold_low = config.contact.threshold;
			config.contact.threshold_high = config.contact.threshold;
			if (contact["threshold_low"]) config.contact.threshold_low = contact["threshold_low"].as<double>();
			if (contact["threshold_high"]) config.contact.threshold_high = contact["threshold_high"].as<double>();
			if (contact["use_hysteresis"]) config.contact.use_hysteresis = contact["use_hysteresis"].as<bool>();
			if (contact["normal"]) {
				config.contact.normal = normalizedVector3FromYaml(contact["normal"], Eigen::Vector3d::UnitZ());
			}
			if (contact["use_force_norm"]) config.contact.use_force_norm = contact["use_force_norm"].as<bool>();
			if (contact["use_absolute_normal_force"]) {
				config.contact.use_absolute_normal_force = contact["use_absolute_normal_force"].as<bool>();
			}
		}

		const YAML::Node grf = root["grf"];
		if (grf) {
			if (grf["force_sign"]) config.grf.force_sign = grf["force_sign"].as<double>();
			if (grf["use_inverse_dynamics_compensation"]) {
				config.grf.use_inverse_dynamics_compensation =
					grf["use_inverse_dynamics_compensation"].as<bool>();
			}
			if (grf["use_joint_acceleration"]) {
				config.grf.use_joint_acceleration = grf["use_joint_acceleration"].as<bool>();
			}
			if (grf["input_filter_alpha"]) config.grf.input_filter_alpha = grf["input_filter_alpha"].as<double>();
			if (grf["force_filter_alpha"]) config.grf.force_filter_alpha = grf["force_filter_alpha"].as<double>();
			if (grf["solve_damping"]) config.grf.solve_damping = grf["solve_damping"].as<double>();
			if (grf["min_singular_value"]) config.grf.min_singular_value = grf["min_singular_value"].as<double>();
			if (grf["max_condition_number"]) config.grf.max_condition_number = grf["max_condition_number"].as<double>();
		}
	} catch (const std::exception& exception) {
		error = exception.what();
		return false;
	}

	if (config.motor_joint_names.size() != 12) {
		error = "robot.motor_joint_names must contain 12 entries";
		return false;
	}
	if (config.motor_indices.size() != config.motor_joint_names.size()) {
		error = "robot.motor_indices must match robot.motor_joint_names";
		return false;
	}
	if (config.feet_frame_names.size() != 4) {
		error = "robot.feet_frame_names must contain 4 entries";
		return false;
	}
	if (config.robot_type.empty()) {
		error = "robot.type cannot be empty";
		return false;
	}

	error.clear();
	return true;
}

} // namespace state_estimator

#endif // STATE_ESTIMATOR_MODELS_ROBOT_CONFIG_HPP
