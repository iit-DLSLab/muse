#ifndef STATE_ESTIMATOR_MODELS_GRF_ESTIMATOR_HPP
#define STATE_ESTIMATOR_MODELS_GRF_ESTIMATOR_HPP

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "state_estimator/Models/unitree_low_state_adapter.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace state_estimator
{

struct GrfEstimatorOptions
{
	std::string urdf_path;
	std::vector<std::string> motor_joint_names;
	std::vector<std::size_t> motor_indices;
	std::vector<std::string> feet_frame_names;
	double force_sign{-1.0};
	bool use_inverse_dynamics_compensation{true};
};

struct GrfEstimate
{
	std::array<Eigen::Vector3d, 4> force{{
		Eigen::Vector3d::Zero(),
		Eigen::Vector3d::Zero(),
		Eigen::Vector3d::Zero(),
		Eigen::Vector3d::Zero()
	}};
	std::array<bool, 4> valid{{false, false, false, false}};

	void reset()
	{
		for (auto& estimated_force : force) {
			estimated_force.setZero();
		}
		valid.fill(false);
	}
};

inline double grfContactMetric(
	const Eigen::Vector3d& force,
	const Eigen::Vector3d& contact_normal,
	bool use_force_norm,
	bool use_absolute_normal_force)
{
	if (use_force_norm) {
		return force.norm();
	}

	const Eigen::Vector3d normal =
		contact_normal.norm() > 1e-9 ? contact_normal.normalized() : Eigen::Vector3d::UnitZ();
	const double normal_force = force.dot(normal);
	return use_absolute_normal_force ? std::abs(normal_force) : normal_force;
}

inline bool fillPinocchioStateFromLowState(
	const pinocchio::Model& model,
	const std::vector<std::string>& motor_joint_names,
	const std::vector<std::size_t>& motor_indices,
	const UnitreeLowState& low_state,
	Eigen::VectorXd& q,
	Eigen::VectorXd& v,
	std::string& error)
{
	q = pinocchio::neutral(model);
	v = Eigen::VectorXd::Zero(model.nv);

	if (motor_indices.size() != motor_joint_names.size()) {
		std::ostringstream stream;
		stream << "motor_indices has " << motor_indices.size()
			   << " entries, but motor_joint_names has " << motor_joint_names.size();
		error = stream.str();
		return false;
	}

	if (motor_joint_names.size() > low_state.motors.size()) {
		std::ostringstream stream;
		stream << "LowState has " << low_state.motors.size()
			   << " motor states, but " << motor_joint_names.size()
			   << " motor joints are configured";
		error = stream.str();
		return false;
	}

	for (std::size_t motor_index = 0; motor_index < motor_joint_names.size(); ++motor_index) {
		const std::string& joint_name = motor_joint_names[motor_index];
		if (!model.existJointName(joint_name)) {
			error = "missing joint '" + joint_name + "'";
			return false;
		}

		const pinocchio::JointIndex joint_id = model.getJointId(joint_name);
		const int q_index = model.joints[joint_id].idx_q();
		const int v_index = model.joints[joint_id].idx_v();
		if (q_index < 0 || q_index >= q.size() || v_index < 0 || v_index >= v.size()) {
			error = "joint '" + joint_name + "' has invalid Pinocchio q/v indices";
			return false;
		}

		const std::size_t low_state_motor_index = motor_indices[motor_index];
		if (low_state_motor_index >= low_state.motors.size()) {
			std::ostringstream stream;
			stream << "configured motor index " << low_state_motor_index
				   << " for joint '" << joint_name
				   << "' is outside LowState motor size " << low_state.motors.size();
			error = stream.str();
			return false;
		}

		const auto& motor_state = low_state.motors[low_state_motor_index];
		const double joint_position = static_cast<double>(motor_state.q);
		const double joint_velocity = static_cast<double>(motor_state.dq);
		if (!std::isfinite(joint_position) || !std::isfinite(joint_velocity)) {
			error = "non-finite joint state for '" + joint_name + "'";
			return false;
		}

		q[q_index] = joint_position;
		v[v_index] = joint_velocity;
	}

	error.clear();
	return true;
}

class GrfEstimator
{
public:
	GrfEstimator() = default;

	bool configure(const GrfEstimatorOptions& options)
	{
		configured_ = false;
		last_error_.clear();
		motor_joint_names_.clear();
		feet_frame_names_.clear();
		q_indices_.fill(-1);
		v_indices_.fill(-1);
		for (auto& leg_indices : leg_v_indices_) {
			leg_indices.fill(-1);
		}

		if (options.motor_joint_names.size() != kMotorJointCount) {
			std::ostringstream stream;
			stream << "expected " << kMotorJointCount << " motor joint names, got "
				   << options.motor_joint_names.size();
			last_error_ = stream.str();
			return false;
		}
		if (options.feet_frame_names.size() != kFootCount) {
			std::ostringstream stream;
			stream << "expected " << kFootCount << " foot frame names, got "
				   << options.feet_frame_names.size();
			last_error_ = stream.str();
			return false;
		}
		if (options.urdf_path.empty()) {
			last_error_ = "empty URDF path";
			return false;
		}

		try {
			pinocchio::urdf::buildModel(options.urdf_path, model_);
			data_ = pinocchio::Data(model_);
		} catch (const std::exception& e) {
			last_error_ = std::string("failed to load URDF '") + options.urdf_path + "': " + e.what();
			return false;
		}

		q_ = pinocchio::neutral(model_);
		v_ = Eigen::VectorXd::Zero(model_.nv);
		a_ = Eigen::VectorXd::Zero(model_.nv);
		tau_ = Eigen::VectorXd::Zero(model_.nv);
		tau_residual_ = Eigen::VectorXd::Zero(model_.nv);

		motor_joint_names_ = options.motor_joint_names;
		motor_indices_ = options.motor_indices.empty()
			? defaultMotorIndices(options.motor_joint_names.size())
			: options.motor_indices;
		feet_frame_names_ = options.feet_frame_names;
		force_sign_ = options.force_sign;
		use_inverse_dynamics_compensation_ = options.use_inverse_dynamics_compensation;

		if (motor_indices_.size() != motor_joint_names_.size()) {
			std::ostringstream stream;
			stream << "motor_indices has " << motor_indices_.size()
				   << " entries, but motor_joint_names has " << motor_joint_names_.size();
			last_error_ = stream.str();
			return false;
		}

		for (std::size_t i = 0; i < motor_joint_names_.size(); ++i) {
			const std::string& joint_name = motor_joint_names_[i];
			if (!model_.existJointName(joint_name)) {
				last_error_ = "missing joint '" + joint_name + "'";
				return false;
			}

			const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
			const int q_index = model_.joints[joint_id].idx_q();
			const int v_index = model_.joints[joint_id].idx_v();
			if (q_index < 0 || q_index >= q_.size() || v_index < 0 || v_index >= v_.size()) {
				last_error_ = "joint '" + joint_name + "' has invalid Pinocchio q/v indices";
				return false;
			}

			q_indices_[i] = q_index;
			v_indices_[i] = v_index;
			leg_v_indices_[i / kJointsPerLeg][i % kJointsPerLeg] = v_index;
		}

		for (const auto& frame_name : feet_frame_names_) {
			if (!model_.existFrame(frame_name)) {
				last_error_ = "missing foot frame '" + frame_name + "'";
				return false;
			}
		}

		configured_ = true;
		last_error_.clear();
		return true;
	}

	bool updateFromLowState(const UnitreeLowState& low_state, GrfEstimate& estimate)
	{
		estimate.reset();

		if (!configured_) {
			last_error_ = "GRF estimator is not configured";
			return false;
		}
		if (motor_indices_.size() != motor_joint_names_.size()) {
			std::ostringstream stream;
			stream << "motor_indices has " << motor_indices_.size()
				   << " entries, but motor_joint_names has " << motor_joint_names_.size();
			last_error_ = stream.str();
			return false;
		}

		q_ = pinocchio::neutral(model_);
		v_.setZero();
		a_.setZero();
		tau_.setZero();

		for (std::size_t i = 0; i < motor_joint_names_.size(); ++i) {
			const std::size_t low_state_motor_index = motor_indices_[i];
			if (low_state_motor_index >= low_state.motors.size()) {
				std::ostringstream stream;
				stream << "configured motor index " << low_state_motor_index
					   << " for joint '" << motor_joint_names_[i]
					   << "' is outside LowState motor size " << low_state.motors.size();
				last_error_ = stream.str();
				return false;
			}

			const auto& motor_state = low_state.motors[low_state_motor_index];
			const double joint_position = static_cast<double>(motor_state.q);
			const double joint_velocity = static_cast<double>(motor_state.dq);
			const double joint_acceleration = static_cast<double>(motor_state.ddq);
			const double joint_torque = static_cast<double>(motor_state.tau_est);

			if (!std::isfinite(joint_position) ||
				!std::isfinite(joint_velocity) ||
				!std::isfinite(joint_acceleration) ||
				!std::isfinite(joint_torque)) {
				last_error_ = "non-finite motor state for '" + motor_joint_names_[i] + "'";
				return false;
			}

			q_[q_indices_[i]] = joint_position;
			v_[v_indices_[i]] = joint_velocity;
			a_[v_indices_[i]] = joint_acceleration;
			tau_[v_indices_[i]] = joint_torque;
		}

		try {
			if (use_inverse_dynamics_compensation_) {
				tau_residual_ = tau_ - pinocchio::rnea(model_, data_, q_, v_, a_);
			} else {
				tau_residual_ = tau_;
			}

			pinocchio::forwardKinematics(model_, data_, q_);
			pinocchio::computeJointJacobians(model_, data_, q_);
			pinocchio::updateFramePlacements(model_, data_);
		} catch (const std::exception& e) {
			last_error_ = std::string("Pinocchio GRF update failed: ") + e.what();
			return false;
		}

		std::ostringstream partial_errors;
		std::size_t valid_count = 0;
		for (std::size_t leg = 0; leg < kFootCount; ++leg) {
			std::string leg_error;
			if (estimateFootForce(leg, estimate.force[leg], leg_error)) {
				estimate.valid[leg] = true;
				++valid_count;
			} else {
				if (partial_errors.tellp() > 0) {
					partial_errors << "; ";
				}
				partial_errors << feet_frame_names_[leg] << ": " << leg_error;
			}
		}

		if (valid_count == kFootCount) {
			last_error_.clear();
			return true;
		}

		last_error_ = partial_errors.str();
		return valid_count > 0;
	}

	const std::string& lastError() const
	{
		return last_error_;
	}

	bool configured() const
	{
		return configured_;
	}

private:
	bool estimateFootForce(std::size_t leg, Eigen::Vector3d& force, std::string& error)
	{
		Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian(6, model_.nv);
		frame_jacobian.setZero();

		try {
			const pinocchio::FrameIndex frame_id = model_.getFrameId(feet_frame_names_[leg]);
			pinocchio::getFrameJacobian(
				model_,
				data_,
				frame_id,
				pinocchio::LOCAL_WORLD_ALIGNED,
				frame_jacobian);
		} catch (const std::exception& e) {
			error = e.what();
			force.setZero();
			return false;
		}

		Eigen::Matrix3d linear_jacobian;
		Eigen::Vector3d joint_torque;
		for (std::size_t i = 0; i < kJointsPerLeg; ++i) {
			const int v_index = leg_v_indices_[leg][i];
			if (v_index < 0 || v_index >= frame_jacobian.cols() || v_index >= tau_residual_.size()) {
				error = "invalid leg Jacobian index";
				force.setZero();
				return false;
			}
			linear_jacobian.col(static_cast<int>(i)) = frame_jacobian.topRows<3>().col(v_index);
			joint_torque(static_cast<int>(i)) = tau_residual_[v_index];
		}

		if (!linear_jacobian.allFinite() || !joint_torque.allFinite()) {
			error = "non-finite Jacobian or torque residual";
			force.setZero();
			return false;
		}

		const Eigen::JacobiSVD<Eigen::Matrix3d> svd(linear_jacobian);
		const auto singular_values = svd.singularValues();
		const double max_singular = singular_values.maxCoeff();
		const double min_singular = singular_values.minCoeff();
		if (!std::isfinite(max_singular) ||
			!std::isfinite(min_singular) ||
			min_singular < kMinSingularValue ||
			max_singular / min_singular > kMaxConditionNumber) {
			error = "ill-conditioned foot Jacobian";
			force.setZero();
			return false;
		}

		force = force_sign_ *
			linear_jacobian.transpose().completeOrthogonalDecomposition().solve(joint_torque);
		if (!force.allFinite()) {
			error = "non-finite estimated force";
			force.setZero();
			return false;
		}

		error.clear();
		return true;
	}

	static constexpr std::size_t kFootCount = 4;
	static constexpr std::size_t kJointsPerLeg = 3;
	static constexpr std::size_t kMotorJointCount = kFootCount * kJointsPerLeg;
	static constexpr double kMinSingularValue = 1e-8;
	static constexpr double kMaxConditionNumber = 1e8;

	pinocchio::Model model_;
	pinocchio::Data data_;
	Eigen::VectorXd q_;
	Eigen::VectorXd v_;
	Eigen::VectorXd a_;
	Eigen::VectorXd tau_;
	Eigen::VectorXd tau_residual_;

	std::vector<std::string> motor_joint_names_;
	std::vector<std::size_t> motor_indices_;
	std::vector<std::string> feet_frame_names_;
	std::array<int, kMotorJointCount> q_indices_{};
	std::array<int, kMotorJointCount> v_indices_{};
	std::array<std::array<int, kJointsPerLeg>, kFootCount> leg_v_indices_{};

	double force_sign_{-1.0};
	bool use_inverse_dynamics_compensation_{true};
	bool configured_{false};
	std::string last_error_;
};

} // namespace state_estimator

#endif // STATE_ESTIMATOR_MODELS_GRF_ESTIMATOR_HPP
