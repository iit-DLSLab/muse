#ifndef STATE_ESTIMATOR_MODELS_CONTACT_ESTIMATOR_HPP
#define STATE_ESTIMATOR_MODELS_CONTACT_ESTIMATOR_HPP

#include "state_estimator/Models/grf_estimator.hpp"
#include "state_estimator/Models/robot_config.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace state_estimator
{

struct ContactEstimate
{
	std::array<bool, 4> stance{{false, false, false, false}};
	std::array<double, 4> metric{{0.0, 0.0, 0.0, 0.0}};
	std::array<double, 4> condition_number{{
		std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::quiet_NaN()
	}};
};

class ContactEstimator
{
public:
	bool configure(const RobotConfig& config)
	{
		config_ = config;
		use_raw_foot_force_ = state_estimator::usesRawFootForce(config.robot_type);
		stance_state_.fill(false);
		last_error_.clear();

		if (use_raw_foot_force_) {
			configured_ = true;
			return true;
		}

		GrfEstimatorOptions options;
		options.urdf_path = config.urdf_path;
		options.motor_joint_names = config.motor_joint_names;
		options.motor_indices = config.motor_indices;
		options.feet_frame_names = config.feet_frame_names;
		options.force_sign = config.grf.force_sign;
		options.use_inverse_dynamics_compensation = config.grf.use_inverse_dynamics_compensation;
		options.use_joint_acceleration = config.grf.use_joint_acceleration;
		options.solve_damping = config.grf.solve_damping;
		options.min_singular_value = config.grf.min_singular_value;
		options.max_condition_number = config.grf.max_condition_number;
		options.input_filter_alpha = config.grf.input_filter_alpha;
		options.force_filter_alpha = config.grf.force_filter_alpha;

		configured_ = grf_estimator_.configure(options);
		if (!configured_) {
			last_error_ = grf_estimator_.lastError();
		}
		return configured_;
	}

	bool update(const UnitreeLowState& low_state, ContactEstimate& estimate)
	{
		estimate = ContactEstimate{};
		if (!configured_) {
			last_error_ = "contact estimator is not configured";
			return false;
		}

		if (use_raw_foot_force_) {
			if (!low_state.has_foot_force) {
				last_error_ = "Go2 LowState does not contain foot_force data";
				stance_state_.fill(false);
				return false;
			}
			for (std::size_t foot = 0; foot < estimate.stance.size(); ++foot) {
				estimate.metric[foot] = std::abs(low_state.foot_force[foot]);
				estimate.stance[foot] = contactFromMetric(foot, estimate.metric[foot]);
				stance_state_[foot] = estimate.stance[foot];
			}
			last_error_.clear();
			return true;
		}

		GrfEstimate grf_estimate;
		if (!grf_estimator_.updateFromLowState(low_state, grf_estimate)) {
			last_error_ = grf_estimator_.lastError();
			stance_state_.fill(false);
			return false;
		}

		for (std::size_t foot = 0; foot < estimate.stance.size(); ++foot) {
			estimate.condition_number[foot] = grf_estimate.condition_number[foot];
			if (!grf_estimate.valid[foot]) {
				stance_state_[foot] = false;
				continue;
			}
			estimate.metric[foot] = grfContactMetric(
				grf_estimate.force[foot],
				config_.contact.normal,
				config_.contact.use_force_norm,
				config_.contact.use_absolute_normal_force);
			estimate.stance[foot] = contactFromMetric(foot, estimate.metric[foot]);
			stance_state_[foot] = estimate.stance[foot];
		}

		last_error_ = grf_estimator_.lastError();
		return true;
	}

	void reset()
	{
		stance_state_.fill(false);
		grf_estimator_.reset();
		last_error_.clear();
	}

	bool configured() const { return configured_; }
	bool usesRawFootForce() const { return use_raw_foot_force_; }
	const std::string& lastError() const { return last_error_; }

private:
	bool contactFromMetric(std::size_t foot, double metric) const
	{
		if (config_.contact.use_hysteresis) {
			return stance_state_[foot]
				? metric > config_.contact.threshold_low
				: metric > config_.contact.threshold_high;
		}
		return metric > config_.contact.threshold;
	}

	RobotConfig config_;
	GrfEstimator grf_estimator_;
	std::array<bool, 4> stance_state_{{false, false, false, false}};
	bool use_raw_foot_force_{false};
	bool configured_{false};
	std::string last_error_;
};

} // namespace state_estimator

#endif // STATE_ESTIMATOR_MODELS_CONTACT_ESTIMATOR_HPP
