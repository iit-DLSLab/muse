#ifndef GRF_CONTACT_ESTIMATOR_HPP
#define GRF_CONTACT_ESTIMATOR_HPP

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "state_estimator/Models/joint_state_snapshot.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace state_estimator
{

struct GrfContactEstimate
{
    std::array<bool, 4> stance{{false, false, false, false}};
    std::array<double, 4> metric{{0.0, 0.0, 0.0, 0.0}};
    std::array<Eigen::Vector3d, 4> force{{
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
    }};
};

class GrfContactEstimator
{
public:
    GrfContactEstimator() = default;

    bool configure(
        const std::string& urdf_path,
        const std::array<std::string, 4>& foot_frame_names,
        const std::array<std::vector<std::string>, 4>& leg_joint_names,
        double threshold,
        double threshold_low,
        double threshold_high,
        bool use_hysteresis,
        bool use_force_norm,
        bool use_absolute_normal_force,
        double force_sign,
        bool use_inverse_dynamics_compensation,
        const Eigen::Vector3d& contact_normal)
    {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        q_ = pinocchio::neutral(model_);
        v_ = Eigen::VectorXd::Zero(model_.nv);
        a_ = Eigen::VectorXd::Zero(model_.nv);
        tau_ = Eigen::VectorXd::Zero(model_.nv);
        tau_residual_ = Eigen::VectorXd::Zero(model_.nv);

	        foot_frame_names_ = foot_frame_names;
	        leg_joint_names_ = leg_joint_names;
	        flattened_joint_names_.clear();
	        for (const auto& joint_names : leg_joint_names_)
	        {
	            flattened_joint_names_.insert(flattened_joint_names_.end(), joint_names.begin(), joint_names.end());
	        }
	        threshold_ = threshold;
	        threshold_low_ = threshold_low;
	        threshold_high_ = threshold_high;
        use_hysteresis_ = use_hysteresis;
        use_force_norm_ = use_force_norm;
        use_absolute_normal_force_ = use_absolute_normal_force;
        force_sign_ = force_sign;
        use_inverse_dynamics_compensation_ = use_inverse_dynamics_compensation;
        contact_normal_ = contact_normal.norm() > 1e-9 ? contact_normal.normalized() : Eigen::Vector3d::UnitZ();

        for (std::size_t leg = 0; leg < foot_frame_names_.size(); ++leg)
        {
            if (!model_.existFrame(foot_frame_names_[leg]))
            {
                last_error_ = "missing foot frame '" + foot_frame_names_[leg] + "'";
                configured_ = false;
                return false;
            }

            if (leg_joint_names_[leg].size() != 3)
            {
                last_error_ = "leg '" + foot_frame_names_[leg] + "' must have exactly 3 joint names";
                configured_ = false;
                return false;
            }

            for (const auto& joint_name : leg_joint_names_[leg])
            {
                if (!model_.existJointName(joint_name))
                {
                    last_error_ = "missing joint '" + joint_name + "'";
                    configured_ = false;
                    return false;
                }
            }
        }

        stance_state_.fill(false);
        configured_ = true;
        last_error_.clear();
        return true;
    }

    bool update(
        const JointStateSnapshot& joints,
        GrfContactEstimate& estimate)
    {
        if (!configured_)
        {
            last_error_ = "estimator is not configured";
            return false;
        }

        std::unordered_map<std::string, double> joint_position;
	        std::unordered_map<std::string, double> joint_velocity;
	        std::unordered_map<std::string, double> joint_acceleration;
	        std::unordered_map<std::string, double> joint_torque;
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
	                use_reading_order_when_names_missing_ &&
	                joint_count == flattened_joint_names_.size() &&
	                joint_index < flattened_joint_names_.size())
	            {
	                name = flattened_joint_names_[joint_index];
	            }

	            if (name.empty()) continue;

	            insertJointValueIfPresent(joint_position, name, joints.position, joint_index);
	            insertJointValueIfPresent(joint_velocity, name, joints.velocity, joint_index);
	            insertJointValueIfPresent(joint_acceleration, name, joints.acceleration, joint_index);
	            insertJointValueIfPresent(joint_torque, name, joints.effort, joint_index);
	        }

        q_ = pinocchio::neutral(model_);
        v_.setZero();
        a_.setZero();
        tau_.setZero();
        std::size_t matched_torques = 0;

        for (pinocchio::JointIndex joint_id = 1; joint_id < model_.njoints; ++joint_id)
        {
            const std::string& joint_name = model_.names[joint_id];
            const int q_index = model_.joints[joint_id].idx_q();
            const int v_index = model_.joints[joint_id].idx_v();

	            double joint_value = 0.0;
	            if (findJointValue(joint_position, joint_name, joint_value) && q_index >= 0 && q_index < q_.size())
	            {
	                q_[q_index] = joint_value;
	            }

	            if (findJointValue(joint_velocity, joint_name, joint_value) && v_index >= 0 && v_index < v_.size())
	            {
	                v_[v_index] = joint_value;
	            }

	            if (findJointValue(joint_acceleration, joint_name, joint_value) && v_index >= 0 && v_index < a_.size())
	            {
	                a_[v_index] = joint_value;
	            }

	            if (findJointValue(joint_torque, joint_name, joint_value) && v_index >= 0 && v_index < tau_.size())
	            {
	                tau_[v_index] = joint_value;
	                matched_torques++;
	            }
	        }

	        if (matched_torques == 0)
	        {
	            last_error_ = unmatchedNamesError(incoming_names, named_readings, joint_count);
	            return false;
	        }

        if (use_inverse_dynamics_compensation_)
        {
            tau_residual_ = tau_ - pinocchio::rnea(model_, data_, q_, v_, a_);
        }
        else
        {
            tau_residual_ = tau_;
        }

        pinocchio::forwardKinematics(model_, data_, q_);
        pinocchio::computeJointJacobians(model_, data_, q_);
        pinocchio::updateFramePlacements(model_, data_);

        for (std::size_t leg = 0; leg < foot_frame_names_.size(); ++leg)
        {
            estimate.force[leg] = estimateFootForce(leg);
            estimate.metric[leg] = contactMetric(estimate.force[leg]);

            if (use_hysteresis_)
            {
                estimate.stance[leg] = stance_state_[leg]
                    ? estimate.metric[leg] > threshold_low_
                    : estimate.metric[leg] > threshold_high_;
            }
            else
            {
                estimate.stance[leg] = estimate.metric[leg] > threshold_;
            }

            stance_state_[leg] = estimate.stance[leg];
        }

        return true;
    }

    const std::string& lastError() const
    {
        return last_error_;
    }

private:
    Eigen::Vector3d estimateFootForce(std::size_t leg) const
    {
        Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian(6, model_.nv);
        frame_jacobian.setZero();

        const pinocchio::FrameIndex frame_id = model_.getFrameId(foot_frame_names_[leg]);
        pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, frame_jacobian);

        Eigen::Matrix3d linear_jacobian;
        Eigen::Vector3d joint_torque;

        for (int i = 0; i < 3; ++i)
        {
            const auto joint_id = model_.getJointId(leg_joint_names_[leg][i]);
            const int v_index = model_.joints[joint_id].idx_v();
            linear_jacobian.col(i) = frame_jacobian.topRows<3>().col(v_index);
            joint_torque(i) = tau_residual_(v_index);
        }

        return force_sign_ * linear_jacobian.transpose().completeOrthogonalDecomposition().solve(joint_torque);
    }

	    double contactMetric(const Eigen::Vector3d& force) const
	    {
	        if (use_force_norm_)
        {
            return force.norm();
        }

	        const double normal_force = force.dot(contact_normal_);
	        return use_absolute_normal_force_ ? std::abs(normal_force) : normal_force;
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
	        error << "no ANYmal joint state names matched the configured URDF joint names";
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
    Eigen::VectorXd a_;
    Eigen::VectorXd tau_;
    Eigen::VectorXd tau_residual_;

	    std::array<std::string, 4> foot_frame_names_;
	    std::array<std::vector<std::string>, 4> leg_joint_names_;
	    std::vector<std::string> flattened_joint_names_;
	    std::array<bool, 4> stance_state_{{false, false, false, false}};

    double threshold_{30.0};
    double threshold_low_{25.0};
    double threshold_high_{35.0};
    bool use_hysteresis_{true};
	    bool use_force_norm_{false};
	    bool use_absolute_normal_force_{true};
	    bool use_inverse_dynamics_compensation_{true};
	    bool use_reading_order_when_names_missing_{true};
	    double force_sign_{-1.0};
    Eigen::Vector3d contact_normal_{Eigen::Vector3d::UnitZ()};
    bool configured_{false};
    std::string last_error_;
};

} // namespace state_estimator

#endif // GRF_CONTACT_ESTIMATOR_HPP
