#ifndef STATE_ESTIMATOR_MODELS_LEG_ODOMETRY_HPP
#define STATE_ESTIMATOR_MODELS_LEG_ODOMETRY_HPP

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "state_estimator/Models/joint_state_snapshot.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <exception>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace state_estimator
{

struct LegKinematics
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LegKinematics()
	{
		for (std::size_t leg = 0; leg < foot_position_base.size(); ++leg)
		{
			foot_position_base[leg].setZero();
			foot_velocity_base[leg].setZero();
			foot_jacobian_base[leg].setZero();
		}
	}

	std::array<Eigen::Vector3d, 4> foot_position_base;
	std::array<Eigen::Vector3d, 4> foot_velocity_base;
	std::array<Eigen::Matrix3d, 4> foot_jacobian_base;
};

struct LegOdometryEstimate
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LegOdometryEstimate()
	{
		for (auto& velocity : base_velocity_from_foot_base)
		{
			velocity.setZero();
		}
		base_velocity_base.setZero();
		base_velocity_world.setZero();
		base_height = 0.0;
		base_height_valid = false;
	}

	LegKinematics kinematics;
	std::array<Eigen::Vector3d, 4> base_velocity_from_foot_base;
	Eigen::Vector3d base_velocity_base;
	Eigen::Vector3d base_velocity_world;
	std::size_t stance_count{0};
	bool base_height_valid{false};
	double base_height{0.0};
};

class LegOdometryModel
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool configure(
		const std::string& urdf_path,
		const std::vector<std::string>& foot_frame_names,
		const std::vector<std::string>& joint_names,
		const std::string& base_frame_name)
	{
		if (foot_frame_names.size() != 4)
		{
			last_error_ = "leg odometry needs exactly 4 foot frame names";
			configured_ = false;
			return false;
		}
		if (joint_names.size() != 12)
		{
			last_error_ = "leg odometry needs exactly 12 joint names";
			configured_ = false;
			return false;
		}

		try
		{
			pinocchio::urdf::buildModel(urdf_path, model_);
		}
		catch (const std::exception& e)
		{
			last_error_ = std::string("failed to load URDF: ") + e.what();
			configured_ = false;
			return false;
		}

		data_ = pinocchio::Data(model_);
		q_ = pinocchio::neutral(model_);
		v_ = Eigen::VectorXd::Zero(model_.nv);
		frame_jacobian_.resize(6, model_.nv);
		frame_jacobian_.setZero();

		foot_frame_names_ = footFrameNamesToArray(foot_frame_names);
		joint_names_ = jointNamesToArray(joint_names);
		flattened_joint_names_ = joint_names;

		for (std::size_t leg = 0; leg < foot_frame_names_.size(); ++leg)
		{
			if (!model_.existFrame(foot_frame_names_[leg]))
			{
				last_error_ = "missing foot frame '" + foot_frame_names_[leg] + "'";
				configured_ = false;
				return false;
			}

			foot_frame_ids_[leg] = model_.getFrameId(foot_frame_names_[leg]);

			for (std::size_t joint = 0; joint < joint_names_[leg].size(); ++joint)
			{
				const std::string& joint_name = joint_names_[leg][joint];
				if (!model_.existJointName(joint_name))
				{
					last_error_ = "missing joint '" + joint_name + "'";
					configured_ = false;
					return false;
				}

				const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
				const int q_index = model_.joints[joint_id].idx_q();
				const int v_index = model_.joints[joint_id].idx_v();
				if (q_index < 0 || q_index >= q_.size() || v_index < 0 || v_index >= v_.size())
				{
					last_error_ = "joint '" + joint_name + "' has invalid Pinocchio q/v indices";
					configured_ = false;
					return false;
				}

				leg_q_indices_[leg][joint] = q_index;
				leg_v_indices_[leg][joint] = v_index;
			}
		}

		has_base_frame_ = !base_frame_name.empty();
		if (has_base_frame_ && !model_.existFrame(base_frame_name))
		{
			last_error_ = "missing base frame '" + base_frame_name + "'";
			configured_ = false;
			return false;
		}
		if (has_base_frame_)
		{
			base_frame_id_ = model_.getFrameId(base_frame_name);
		}

		configured_ = true;
		last_error_.clear();
		return true;
	}

	bool update(
		const JointStateSnapshot& joints,
		const Eigen::Vector3d& omega_base,
		const std::array<bool, 4>& stance,
		const Eigen::Matrix3d& world_R_base,
		LegOdometryEstimate& estimate)
	{
		if (!configured_)
		{
			last_error_ = "leg odometry model is not configured";
			return false;
		}

		if (!fillStateFromJointState(joints))
		{
			return false;
		}

		computeKinematics(estimate.kinematics);

		estimate.base_velocity_base.setZero();
		estimate.base_velocity_world.setZero();
		estimate.stance_count = 0;
		estimate.base_height = 0.0;
		estimate.base_height_valid = false;

		for (std::size_t leg = 0; leg < estimate.base_velocity_from_foot_base.size(); ++leg)
		{
			estimate.base_velocity_from_foot_base[leg] = -(
				estimate.kinematics.foot_velocity_base[leg] +
				omega_base.cross(estimate.kinematics.foot_position_base[leg])
			);

			if (stance[leg])
			{
				estimate.base_velocity_base += estimate.base_velocity_from_foot_base[leg];
				estimate.base_height -= (world_R_base * estimate.kinematics.foot_position_base[leg]).z();
				estimate.stance_count++;
			}
		}

		const double stance_denominator = estimate.stance_count > 0
			? static_cast<double>(estimate.stance_count)
			: 1.0;
		estimate.base_velocity_base /= (stance_denominator + 1e-5);
		estimate.base_velocity_world = world_R_base * estimate.base_velocity_base;
		if (estimate.stance_count > 0)
		{
			estimate.base_height /= stance_denominator;
			estimate.base_height_valid = true;
		}

		last_error_.clear();
		return true;
	}

	const std::string& lastError() const
	{
		return last_error_;
	}

private:
	bool fillStateFromJointState(const JointStateSnapshot& joints)
	{
		std::unordered_map<std::string, double> joint_position;
		std::unordered_map<std::string, double> joint_velocity;
		std::vector<std::string> incoming_names;
		std::size_t named_readings = 0;
		const std::size_t joint_count = maxJointFieldSize(joints);

		for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index)
		{
			std::string name = joint_index < joints.name.size() ? joints.name[joint_index] : std::string();
			if (!name.empty())
			{
				named_readings++;
				incoming_names.push_back(name);
			}
			else if (
				joint_count == flattened_joint_names_.size() &&
				joint_index < flattened_joint_names_.size())
			{
				name = flattened_joint_names_[joint_index];
			}

			if (name.empty()) continue;

			insertJointValueIfPresent(joint_position, name, joints.position, joint_index);
			insertJointValueIfPresent(joint_velocity, name, joints.velocity, joint_index);
		}

		q_ = pinocchio::neutral(model_);
		v_.setZero();

		std::size_t matched_positions = 0;
		std::size_t matched_velocities = 0;
		for (std::size_t leg = 0; leg < joint_names_.size(); ++leg)
		{
			for (std::size_t joint = 0; joint < joint_names_[leg].size(); ++joint)
			{
				const std::string& joint_name = joint_names_[leg][joint];
				double joint_value = 0.0;
				if (findJointValue(joint_position, joint_name, joint_value))
				{
					q_[leg_q_indices_[leg][joint]] = joint_value;
					matched_positions++;
				}
				if (findJointValue(joint_velocity, joint_name, joint_value))
				{
					v_[leg_v_indices_[leg][joint]] = joint_value;
					matched_velocities++;
				}
			}
		}

		const std::size_t expected_joints = flattened_joint_names_.size();
		if (matched_positions != expected_joints || matched_velocities != expected_joints)
		{
			std::ostringstream error;
			error << unmatchedNamesError(incoming_names, named_readings, joint_count)
			      << "; matched positions " << matched_positions << "/" << expected_joints
			      << ", velocities " << matched_velocities << "/" << expected_joints;
			last_error_ = error.str();
			return false;
		}

		return true;
	}

	void computeKinematics(LegKinematics& kinematics)
	{
		pinocchio::forwardKinematics(model_, data_, q_, v_);
		pinocchio::computeJointJacobians(model_, data_, q_);
		pinocchio::updateFramePlacements(model_, data_);

		Eigen::Matrix3d world_R_base = Eigen::Matrix3d::Identity();
		Eigen::Vector3d world_p_base = Eigen::Vector3d::Zero();
		if (has_base_frame_)
		{
			world_R_base = data_.oMf[base_frame_id_].rotation();
			world_p_base = data_.oMf[base_frame_id_].translation();
		}
		const Eigen::Matrix3d base_R_world = world_R_base.transpose();

		for (std::size_t leg = 0; leg < foot_frame_ids_.size(); ++leg)
		{
			const pinocchio::FrameIndex foot_frame_id = foot_frame_ids_[leg];
			kinematics.foot_position_base[leg] =
				base_R_world * (data_.oMf[foot_frame_id].translation() - world_p_base);

			frame_jacobian_.setZero();
			pinocchio::getFrameJacobian(
				model_,
				data_,
				foot_frame_id,
				pinocchio::LOCAL_WORLD_ALIGNED,
				frame_jacobian_);

			Eigen::Matrix3d foot_jacobian_world;
			for (std::size_t joint = 0; joint < 3; ++joint)
			{
				foot_jacobian_world.col(joint) = frame_jacobian_.topRows<3>().col(leg_v_indices_[leg][joint]);
			}

			kinematics.foot_jacobian_base[leg] = base_R_world * foot_jacobian_world;

			Eigen::Vector3d leg_velocity;
			for (std::size_t joint = 0; joint < 3; ++joint)
			{
				leg_velocity[joint] = v_[leg_v_indices_[leg][joint]];
			}
			kinematics.foot_velocity_base[leg] = kinematics.foot_jacobian_base[leg] * leg_velocity;
		}
	}

	static std::array<std::string, 4> footFrameNamesToArray(const std::vector<std::string>& names)
	{
		return {{names[0], names[1], names[2], names[3]}};
	}

	static std::array<std::array<std::string, 3>, 4> jointNamesToArray(const std::vector<std::string>& names)
	{
		return {{
			{{names[0], names[1], names[2]}},
			{{names[3], names[4], names[5]}},
			{{names[6], names[7], names[8]}},
			{{names[9], names[10], names[11]}}
		}};
	}

	static void insertJointValue(std::unordered_map<std::string, double>& values, const std::string& name, double value)
	{
		values[name] = value;
		const std::string normalized_name = normalizeJointName(name);
		if (!normalized_name.empty())
		{
			values[normalized_name] = value;
		}
	}

	static void insertJointValueIfPresent(
		std::unordered_map<std::string, double>& values,
		const std::string& name,
		const std::vector<double>& source,
		std::size_t index)
	{
		if (index < source.size())
		{
			insertJointValue(values, name, source[index]);
		}
	}

	static std::size_t maxJointFieldSize(const JointStateSnapshot& joints)
	{
		return std::max({
			joints.name.size(),
			joints.position.size(),
			joints.velocity.size(),
			joints.acceleration.size(),
			joints.effort.size()
		});
	}

	static bool findJointValue(
		const std::unordered_map<std::string, double>& values,
		const std::string& joint_name,
		double& value)
	{
		auto value_it = values.find(joint_name);
		if (value_it != values.end())
		{
			value = value_it->second;
			return true;
		}

		value_it = values.find(normalizeJointName(joint_name));
		if (value_it != values.end())
		{
			value = value_it->second;
			return true;
		}

		return false;
	}

	static std::string normalizeJointName(std::string name)
	{
		trim(name);

		const std::size_t slash_pos = name.find_last_of("/\\");
		if (slash_pos != std::string::npos)
		{
			name = name.substr(slash_pos + 1);
		}

		const std::size_t namespace_pos = name.rfind("::");
		if (namespace_pos != std::string::npos)
		{
			name = name.substr(namespace_pos + 2);
		}

		std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
			return static_cast<char>(std::toupper(c));
		});

		stripSuffix(name, "_JOINT");
		stripSuffix(name, "_MOTOR");
		stripPrefix(name, "JOINT_");

		return name;
	}

	static void trim(std::string& value)
	{
		value.erase(
			value.begin(),
			std::find_if(value.begin(), value.end(), [](unsigned char c) { return !std::isspace(c); })
		);
		value.erase(
			std::find_if(value.rbegin(), value.rend(), [](unsigned char c) { return !std::isspace(c); }).base(),
			value.end()
		);
	}

	static void stripSuffix(std::string& value, const std::string& suffix)
	{
		if (value.size() >= suffix.size() && value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0)
		{
			value.erase(value.size() - suffix.size());
		}
	}

	static void stripPrefix(std::string& value, const std::string& prefix)
	{
		if (value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0)
		{
			value.erase(0, prefix.size());
		}
	}

	std::string unmatchedNamesError(
		const std::vector<std::string>& incoming_names,
		std::size_t named_readings,
		std::size_t total_readings) const
	{
		std::ostringstream error;
		error << "ANYmal joint state did not cover all configured leg joint position/velocity names";
		error << "; expected joints include " << sampleNames(flattened_joint_names_, 6);
		if (named_readings == 0)
		{
			error << "; all " << total_readings << " joint names are empty";
			if (total_readings != flattened_joint_names_.size())
			{
				error << " and reading-order fallback requires " << flattened_joint_names_.size() << " joints";
			}
		}
		else
		{
			error << "; incoming names include " << sampleNames(incoming_names, 6);
		}
		return error.str();
	}

	static std::string sampleNames(const std::vector<std::string>& names, std::size_t max_count)
	{
		std::ostringstream sample;
		sample << "[";
		const std::size_t count = std::min(max_count, names.size());
		for (std::size_t i = 0; i < count; ++i)
		{
			if (i > 0) sample << ", ";
			sample << names[i];
		}
		if (names.size() > max_count) sample << ", ...";
		sample << "]";
		return sample.str();
	}

	pinocchio::Model model_;
	mutable pinocchio::Data data_;
	Eigen::VectorXd q_;
	Eigen::VectorXd v_;
	Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian_;

	std::array<std::string, 4> foot_frame_names_;
	std::array<pinocchio::FrameIndex, 4> foot_frame_ids_;
	std::array<std::array<std::string, 3>, 4> joint_names_;
	std::array<std::array<int, 3>, 4> leg_q_indices_;
	std::array<std::array<int, 3>, 4> leg_v_indices_;
	std::vector<std::string> flattened_joint_names_;

	pinocchio::FrameIndex base_frame_id_{0};
	bool has_base_frame_{false};
	bool configured_{false};
	std::string last_error_;
};

} // namespace state_estimator

#endif // STATE_ESTIMATOR_MODELS_LEG_ODOMETRY_HPP
