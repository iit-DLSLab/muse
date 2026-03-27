#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>

#include "state_estimator_msgs/msg/contact_detection.hpp"
#include "unitree_go/msg/low_state.hpp"

#include <yaml-cpp/yaml.h>


namespace state_estimator_plugins
{

	class ContactDetectionPlugin : public PluginBase
	{
	public:
		ContactDetectionPlugin() = default;
		~ContactDetectionPlugin() = default;

		std::string getName() override { return std::string("ContactDetection"); }
		std::string getDescription() override { return std::string("Contact Detection Plugin"); }

		void initialize_() override {

			// Load parameters from YAML
			std::string low_state_topic = "/lowstate";
			std::string pub_topic       = "contact_detection";
			contact_threshold_ = 30.0;

			if (!config_dir_.empty()) {
				try {
					YAML::Node cfg = YAML::LoadFile(config_dir_ + "/contact_plugin.yaml")["contact_detection_plugin"];
					if (cfg["low_state_topic"])      low_state_topic  = cfg["low_state_topic"].as<std::string>();
					if (cfg["pub_topic"])            pub_topic        = cfg["pub_topic"].as<std::string>();
					if (cfg["contact_force_threshold"]) contact_threshold_ = cfg["contact_force_threshold"].as<double>();
				} catch (const std::exception& e) {
					RCLCPP_WARN(node_->get_logger(), "Could not load contact config: %s", e.what());
				}
			}

			RCLCPP_INFO(node_->get_logger(), "ContactDetectionPlugin: contact threshold = %.2f", contact_threshold_);

			// Subscriber
			low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
				low_state_topic, 250,
				std::bind(&ContactDetectionPlugin::callback, this, std::placeholders::_1));

			// Publisher
			pub_ = node_->create_publisher<state_estimator_msgs::msg::ContactDetection>(pub_topic, 250);
		}


		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback(const unitree_go::msg::LowState::SharedPtr low_state)
		{
			// Threshold raw foot force scalars from LowState (FL=0, FR=1, RL=2, RR=3)
			msg_.stance_lf = std::abs(low_state->foot_force[0]) > contact_threshold_;
			msg_.stance_rf = std::abs(low_state->foot_force[1]) > contact_threshold_;
			msg_.stance_lh = std::abs(low_state->foot_force[2]) > contact_threshold_;
			msg_.stance_rh = std::abs(low_state->foot_force[3]) > contact_threshold_;

			// publishing
			msg_.header.stamp = node_->now();
			pub_->publish(msg_);

		} // end callback


	private:

		rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
		rclcpp::Publisher<state_estimator_msgs::msg::ContactDetection>::SharedPtr pub_;

		state_estimator_msgs::msg::ContactDetection msg_;

		double contact_threshold_{30.0};



	}; // end class ContactDetectionPlugin

} //end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ContactDetectionPlugin, state_estimator_plugins::PluginBase)
