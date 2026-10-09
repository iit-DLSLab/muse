#include "state_estimator/Models/contact_estimator.hpp"
#include "state_estimator/Models/robot_config.hpp"
#include "state_estimator/Models/unitree_low_state_adapter.hpp"
#include "state_estimator/plugin.hpp"

#include "state_estimator_msgs/msg/contact_detection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <exception>
#include <stdexcept>
#include <string>

namespace state_estimator_plugins
{

class ContactDetectionPlugin : public PluginBase
{
public:
	std::string getName() override { return std::string("ContactDetection"); }
	std::string getDescription() override { return std::string("Contact Detection Plugin"); }

	void initialize_() override
	{
		state_estimator::RobotConfig robot_config;
		std::string config_error;
		if (!state_estimator::loadRobotConfig(config_dir_, robot_config, config_error)) {
			throw std::runtime_error("ContactDetectionPlugin: " + config_error);
		}

		std::string pub_topic = "contact_detection";
		try {
			const YAML::Node config =
				YAML::LoadFile(config_dir_ + "/contact_plugin.yaml")["contact_detection_plugin"];
			if (config["pub_topic"]) pub_topic = config["pub_topic"].as<std::string>();
		} catch (const std::exception& exception) {
			RCLCPP_WARN(node_->get_logger(), "Could not load contact plugin config: %s", exception.what());
		}

		if (!contact_estimator_.configure(robot_config)) {
			throw std::runtime_error(
				"ContactDetectionPlugin: " + contact_estimator_.lastError());
		}

		pub_ = node_->create_publisher<state_estimator_msgs::msg::ContactDetection>(pub_topic, 250);
		createLowStateSubscription(robot_config.robot_type, robot_config.low_state_topic);

		RCLCPP_INFO(node_->get_logger(),
			"ContactDetectionPlugin: robot=%s, source=%s, threshold=%.2f, hysteresis=%s [low=%.2f high=%.2f]",
			robot_config.robot_type.c_str(),
			contact_estimator_.usesRawFootForce() ? "LowState.foot_force" : "filtered Pinocchio GRF",
			robot_config.contact.threshold,
			robot_config.contact.use_hysteresis ? "true" : "false",
			robot_config.contact.threshold_low,
			robot_config.contact.threshold_high);
	}

	void shutdown_() override {}
	void pause_() override {}
	void resume_() override {}
	void reset_() override { contact_estimator_.reset(); }

private:
	void createLowStateSubscription(
		const std::string& robot_type,
		const std::string& low_state_topic)
	{
		if (state_estimator::messageFamilyForRobotType(robot_type) ==
			state_estimator::UnitreeMessageFamily::Go) {
			go_low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
				low_state_topic, 250,
				std::bind(&ContactDetectionPlugin::callbackGo, this, std::placeholders::_1));
			return;
		}

		hg_low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
			low_state_topic, 250,
			std::bind(&ContactDetectionPlugin::callbackHg, this, std::placeholders::_1));
	}

	template <typename RosLowState>
	void adaptAndProcess(const RosLowState& low_state, const char* message_type)
	{
		std::string error;
		if (!state_estimator::fromRos(low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: failed to adapt %s LowState: %s",
				message_type, error.c_str());
			publishContact({{false, false, false, false}});
			return;
		}
		processLowState(normalized_low_state_);
	}

	void callbackGo(const unitree_go::msg::LowState::SharedPtr low_state)
	{
		adaptAndProcess(*low_state, "unitree_go");
	}

	void callbackHg(const unitree_hg::msg::LowState::SharedPtr low_state)
	{
		adaptAndProcess(*low_state, "unitree_hg");
	}

	void processLowState(const state_estimator::UnitreeLowState& low_state)
	{
		state_estimator::ContactEstimate estimate;
		if (!contact_estimator_.update(low_state, estimate)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: contact update failed: %s",
				contact_estimator_.lastError().c_str());
			publishContact({{false, false, false, false}});
			return;
		}

		if (!contact_estimator_.lastError().empty()) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ContactDetectionPlugin: partial contact estimate: %s",
				contact_estimator_.lastError().c_str());
		}
		RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
			"Contact metrics [FL FR RL RR]: %.3f %.3f %.3f %.3f",
			estimate.metric[0], estimate.metric[1], estimate.metric[2], estimate.metric[3]);
		publishContact(estimate.stance);
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

	rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr go_low_state_sub_;
	rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr hg_low_state_sub_;
	rclcpp::Publisher<state_estimator_msgs::msg::ContactDetection>::SharedPtr pub_;
	state_estimator_msgs::msg::ContactDetection msg_;
	state_estimator::UnitreeLowState normalized_low_state_;
	state_estimator::ContactEstimator contact_estimator_;
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ContactDetectionPlugin, state_estimator_plugins::PluginBase)
