#include "state_estimator/Models/grf_estimator.hpp"
#include "state_estimator/Models/unitree_low_state_adapter.hpp"
#include "state_estimator/plugin.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "state_estimator_msgs/msg/contact_detection.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <exception>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class ContactDetectionPlugin : public PluginBase
{
public:
	ContactDetectionPlugin() = default;
	~ContactDetectionPlugin() override = default;

	std::string getName() override { return std::string("ContactDetection"); }
	std::string getDescription() override { return std::string("Contact Detection Plugin"); }

	void initialize_() override
	{
		std::string low_state_topic = "/lowstate";
		std::string pub_topic = "contact_detection";
		std::string urdf_path = "$(find state_estimator)/urdfs/go2.urdf";
		std::string robot_type = "go2";
		std::vector<std::string> motor_joint_names = defaultMotorJointNames();
		std::vector<std::size_t> motor_indices = state_estimator::defaultMotorIndices(motor_joint_names.size());
		std::vector<std::string> feet_frame_names = defaultFeetFrameNames();
		std::vector<double> contact_normal_vec{0.0, 0.0, 1.0};
		grf_threshold_ = 15.0;
		grf_threshold_low_ = grf_threshold_;
		grf_threshold_high_ = grf_threshold_;
		double force_sign = -1.0;
		bool use_inverse_dynamics_compensation = true;

		if (!config_dir_.empty()) {
			try {
				YAML::Node cfg = YAML::LoadFile(config_dir_ + "/contact_plugin.yaml")["contact_detection_plugin"];
				if (cfg["robot_type"]) robot_type = cfg["robot_type"].as<std::string>();
				if (cfg["low_state_topic"]) low_state_topic = cfg["low_state_topic"].as<std::string>();
				if (cfg["pub_topic"]) pub_topic = cfg["pub_topic"].as<std::string>();
				if (cfg["contact_force_threshold"]) grf_threshold_ = cfg["contact_force_threshold"].as<double>();
				if (cfg["grf_threshold"]) grf_threshold_ = cfg["grf_threshold"].as<double>();
				grf_threshold_low_ = grf_threshold_;
				grf_threshold_high_ = grf_threshold_;
				if (cfg["threshold_low"]) grf_threshold_low_ = cfg["threshold_low"].as<double>();
				if (cfg["threshold_high"]) grf_threshold_high_ = cfg["threshold_high"].as<double>();
				if (cfg["contact_force_threshold_low"]) {
					grf_threshold_low_ = cfg["contact_force_threshold_low"].as<double>();
				}
				if (cfg["contact_force_threshold_high"]) {
					grf_threshold_high_ = cfg["contact_force_threshold_high"].as<double>();
				}
				if (cfg["grf_threshold_low"]) grf_threshold_low_ = cfg["grf_threshold_low"].as<double>();
				if (cfg["grf_threshold_high"]) grf_threshold_high_ = cfg["grf_threshold_high"].as<double>();
				if (cfg["use_hysteresis"]) use_grf_hysteresis_ = cfg["use_hysteresis"].as<bool>();
				if (cfg["use_grf_hysteresis"]) use_grf_hysteresis_ = cfg["use_grf_hysteresis"].as<bool>();
				if (cfg["urdf_path"]) urdf_path = cfg["urdf_path"].as<std::string>();
				if (cfg["motor_joint_names"]) motor_joint_names = cfg["motor_joint_names"].as<std::vector<std::string>>();
				if (cfg["motor_indices"]) motor_indices = toSizeTVector(cfg["motor_indices"].as<std::vector<int>>());
				if (cfg["feet_frame_names"]) feet_frame_names = cfg["feet_frame_names"].as<std::vector<std::string>>();
				if (cfg["contact_normal"]) contact_normal_vec = cfg["contact_normal"].as<std::vector<double>>();
				if (cfg["use_force_norm"]) use_force_norm_ = cfg["use_force_norm"].as<bool>();
				if (cfg["use_absolute_normal_force"]) {
					use_absolute_normal_force_ = cfg["use_absolute_normal_force"].as<bool>();
				}
				if (cfg["force_sign"]) force_sign = cfg["force_sign"].as<double>();
				if (cfg["use_inverse_dynamics_compensation"]) {
					use_inverse_dynamics_compensation = cfg["use_inverse_dynamics_compensation"].as<bool>();
				}
			} catch (const std::exception& e) {
				RCLCPP_WARN(node_->get_logger(), "Could not load contact config: %s", e.what());
			}
		}

		if (motor_joint_names.size() != 12) {
			RCLCPP_WARN(node_->get_logger(),
				"ContactDetectionPlugin: motor_joint_names must contain 12 entries; using Go2 defaults");
			motor_joint_names = defaultMotorJointNames();
		}
		if (motor_indices.size() != motor_joint_names.size()) {
			RCLCPP_WARN(node_->get_logger(),
				"ContactDetectionPlugin: motor_indices must match motor_joint_names; using sequential defaults");
			motor_indices = state_estimator::defaultMotorIndices(motor_joint_names.size());
		}
		if (feet_frame_names.size() != 4) {
			RCLCPP_WARN(node_->get_logger(),
				"ContactDetectionPlugin: feet_frame_names must contain 4 entries; using Go2 defaults");
			feet_frame_names = defaultFeetFrameNames();
		}

		contact_normal_ = vector3FromConfig(contact_normal_vec, Eigen::Vector3d::UnitZ());

		state_estimator::GrfEstimatorOptions grf_options;
		grf_options.urdf_path = resolvePackagePath(urdf_path);
		grf_options.motor_joint_names = motor_joint_names;
		grf_options.motor_indices = motor_indices;
		grf_options.feet_frame_names = feet_frame_names;
		grf_options.force_sign = force_sign;
		grf_options.use_inverse_dynamics_compensation = use_inverse_dynamics_compensation;

		grf_estimator_configured_ = grf_estimator_.configure(grf_options);
		if (!grf_estimator_configured_) {
			RCLCPP_ERROR(node_->get_logger(),
				"ContactDetectionPlugin: failed to configure GRF estimator: %s",
				grf_estimator_.lastError().c_str());
		}

		pub_ = node_->create_publisher<state_estimator_msgs::msg::ContactDetection>(pub_topic, 250);
		createLowStateSubscription(robot_type, low_state_topic);

		RCLCPP_INFO(node_->get_logger(),
			"ContactDetectionPlugin: robot_type=%s, GRF threshold=%.2f, hysteresis=%s [low=%.2f high=%.2f], URDF=%s",
			robot_type.c_str(), grf_threshold_,
			use_grf_hysteresis_ ? "true" : "false",
			grf_threshold_low_, grf_threshold_high_,
			grf_options.urdf_path.c_str());
	}

	void shutdown_() override {}
	void pause_() override {}
	void resume_() override {}
	void reset_() override {}

private:
	void createLowStateSubscription(const std::string& robot_type, const std::string& low_state_topic)
	{
		const auto family = state_estimator::messageFamilyForRobotType(robot_type);
		if (family == state_estimator::UnitreeMessageFamily::Go) {
			go_low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
				low_state_topic, 250,
				std::bind(&ContactDetectionPlugin::callbackGo, this, std::placeholders::_1));
			return;
		}

		hg_low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
			low_state_topic, 250,
			std::bind(&ContactDetectionPlugin::callbackHg, this, std::placeholders::_1));
	}

	void callbackGo(const unitree_go::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: failed to adapt unitree_go LowState: %s", error.c_str());
			publishContact({{false, false, false, false}});
			return;
		}
		processLowState(normalized_low_state_);
	}

	void callbackHg(const unitree_hg::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: failed to adapt unitree_hg LowState: %s", error.c_str());
			publishContact({{false, false, false, false}});
			return;
		}
		processLowState(normalized_low_state_);
	}

	void processLowState(const state_estimator::UnitreeLowState& low_state)
	{
		std::array<bool, 4> stance{{false, false, false, false}};
		std::array<double, 4> metrics{{0.0, 0.0, 0.0, 0.0}};

		if (!grf_estimator_configured_) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: GRF estimator is not configured; publishing no contact");
			resetStanceState();
			publishContact(stance);
			return;
		}

		state_estimator::GrfEstimate estimate;
		const bool updated = grf_estimator_.updateFromLowState(low_state, estimate);
		if (!updated) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: GRF estimator update failed: %s",
				grf_estimator_.lastError().c_str());
			resetStanceState();
			publishContact(stance);
			return;
		}

		for (std::size_t i = 0; i < stance.size(); ++i) {
			if (!estimate.valid[i]) {
				stance_state_[i] = false;
				continue;
			}
			metrics[i] = state_estimator::grfContactMetric(
				estimate.force[i],
				contact_normal_,
				use_force_norm_,
				use_absolute_normal_force_);
			stance[i] = contactFromMetric(i, metrics[i]);
			stance_state_[i] = stance[i];
		}

		if (!grf_estimator_.lastError().empty()) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: partial GRF estimate: %s",
				grf_estimator_.lastError().c_str());
		}
		RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
			"ContactDetectionPlugin GRF metrics [FL FR RL RR]: %.3f %.3f %.3f %.3f",
			metrics[0], metrics[1], metrics[2], metrics[3]);

		publishContact(stance);
	}

	void publishContact(const std::array<bool, 4>& stance)
	{
		msg_.stance_lf = stance[0];
		msg_.stance_rf = stance[1];
		msg_.stance_lh = stance[2];
		msg_.stance_rh = stance[3];
		msg_.header.stamp = node_->now();
		pub_->publish(msg_);
	}

	bool contactFromMetric(std::size_t foot_index, double metric) const
	{
		if (use_grf_hysteresis_) {
			return stance_state_[foot_index] ? metric > grf_threshold_low_ : metric > grf_threshold_high_;
		}
		return metric > grf_threshold_;
	}

	void resetStanceState()
	{
		stance_state_.fill(false);
	}

	static std::vector<std::string> defaultMotorJointNames()
	{
		return {
			"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
			"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
			"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
			"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
		};
	}

	static std::vector<std::string> defaultFeetFrameNames()
	{
		return {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
	}

	static std::vector<std::size_t> toSizeTVector(const std::vector<int>& values)
	{
		std::vector<std::size_t> converted;
		converted.reserve(values.size());
		for (const int value : values) {
			if (value < 0) {
				continue;
			}
			converted.push_back(static_cast<std::size_t>(value));
		}
		return converted;
	}

	static Eigen::Vector3d vector3FromConfig(
		const std::vector<double>& values,
		const Eigen::Vector3d& fallback)
	{
		if (values.size() != 3) {
			return fallback;
		}
		const Eigen::Vector3d vector(values[0], values[1], values[2]);
		if (!vector.allFinite() || vector.norm() < 1e-9) {
			return fallback;
		}
		return vector.normalized();
	}

	static std::string resolvePackagePath(const std::string& input)
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
		try {
			const std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
			resolved.replace(token_pos, package_end - token_pos + 1, package_path);
		} catch (const std::exception&) {
			return input;
		}
		return resolved;
	}

	rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr go_low_state_sub_;
	rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr hg_low_state_sub_;
	rclcpp::Publisher<state_estimator_msgs::msg::ContactDetection>::SharedPtr pub_;

	state_estimator_msgs::msg::ContactDetection msg_;
	state_estimator::UnitreeLowState normalized_low_state_;
	state_estimator::GrfEstimator grf_estimator_;

	bool grf_estimator_configured_{false};
	double grf_threshold_{15.0};
	double grf_threshold_low_{15.0};
	double grf_threshold_high_{15.0};
	bool use_grf_hysteresis_{false};
	std::array<bool, 4> stance_state_{{false, false, false, false}};
	Eigen::Vector3d contact_normal_{Eigen::Vector3d::UnitZ()};
	bool use_force_norm_{false};
	bool use_absolute_normal_force_{true};
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ContactDetectionPlugin, state_estimator_plugins::PluginBase)
