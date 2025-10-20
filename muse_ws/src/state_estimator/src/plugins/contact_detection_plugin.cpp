// ROS 2 version of the ContactDetection plugin
#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>

#include "state_estimator_msgs/msg/contact_detection.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <message_filters/time_synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>


namespace state_estimator_plugins
{

using Wrench = geometry_msgs::msg::WrenchStamped;
using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<Wrench, Wrench, Wrench, Wrench>;
using ExactTimePolicy = message_filters::sync_policies::ExactTime<Wrench, Wrench, Wrench, Wrench>;

#define MySyncPolicy ApproximateTimePolicy

	class ContactDetectionPlugin : public PluginBase
	{
	public:
		ContactDetectionPlugin() = default;

		~ContactDetectionPlugin() override = default;

		std::string getName() override { return std::string("ContactDetection"); }
		std::string getDescription() override { return std::string("Contact Detection Plugin"); }

		void initialize_() override {

		auto node = this->node_;
		if (!node) {
		    throw std::runtime_error("ContactDetectionPlugin: node_ is null");
		}

		// Parameters (ROS 2)
		// LF: left front leg, RF: right front leg, LH: left hind leg, RH: right hind leg
		const std::string lf_topic = node->declare_parameter<std::string>("contact_detection_plugin.wrench_lf_topic", "/state_estimator/contact_force_lf_foot");
		const std::string rf_topic = node->declare_parameter<std::string>("contact_detection_plugin.wrench_rf_topic", "/state_estimator/contact_force_rf_foot");
		const std::string lh_topic = node->declare_parameter<std::string>("contact_detection_plugin.wrench_lh_topic", "/state_estimator/contact_force_lh_foot");
		const std::string rh_topic = node->declare_parameter<std::string>("contact_detection_plugin.wrench_rh_topic", "/state_estimator/contact_force_rh_foot");
		const std::string pub_topic = node->declare_parameter<std::string>("contact_detection_plugin.pub_topic", "contact_detection");
		grf_threshold_ = node->declare_parameter<double>("contact_detection_plugin.grf_threshold", 15.0);

		RCLCPP_INFO(node->get_logger(), "ContactDetectionPlugin loaded with GRF threshold: %.3f", grf_threshold_);

		// QoS profile for sensor data
		auto sensor_qos = rclcpp::SensorDataQoS();
		auto rmw_qos = sensor_qos.get_rmw_qos_profile();

		// Set up subscribers (ROS 2 message_filters requires node interfaces and rmw_qos_profile)
		wrench_lf_sub_ = std::make_shared<message_filters::Subscriber<Wrench>>(node, lf_topic, rmw_qos);
		wrench_rf_sub_ = std::make_shared<message_filters::Subscriber<Wrench>>(node, rf_topic, rmw_qos);
		wrench_lh_sub_ = std::make_shared<message_filters::Subscriber<Wrench>>(node, lh_topic, rmw_qos);
		wrench_rh_sub_ = std::make_shared<message_filters::Subscriber<Wrench>>(node, rh_topic, rmw_qos);

		// Synchronizer
		sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(250), *wrench_lf_sub_, *wrench_rf_sub_, *wrench_lh_sub_, *wrench_rh_sub_);
		sync_->registerCallback(std::bind(&ContactDetectionPlugin::callback, this,
							    std::placeholders::_1,
							    std::placeholders::_2,
							    std::placeholders::_3,
							    std::placeholders::_4));

		// Publisher
		pub_ = node->create_publisher<state_estimator_msgs::msg::ContactDetection>(pub_topic, rclcpp::QoS(10));

		}


		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback
		(
			const Wrench::ConstSharedPtr wrench_lf,
			const Wrench::ConstSharedPtr wrench_rf,
			const Wrench::ConstSharedPtr wrench_lh,
			const Wrench::ConstSharedPtr wrench_rh
		)
		{
			// Calculate the norm of the wrenches
            wrench_lf_norm = sqrt(pow(wrench_lf->wrench.force.x,2) + pow(wrench_lf->wrench.force.y,2) + pow(wrench_lf->wrench.force.z,2));
            wrench_rf_norm = sqrt(pow(wrench_rf->wrench.force.x,2) + pow(wrench_rf->wrench.force.y,2) + pow(wrench_rf->wrench.force.z,2));
            wrench_lh_norm = sqrt(pow(wrench_lh->wrench.force.x,2) + pow(wrench_lh->wrench.force.y,2) + pow(wrench_lh->wrench.force.z,2));
            wrench_rh_norm = sqrt(pow(wrench_rh->wrench.force.x,2) + pow(wrench_rh->wrench.force.y,2) + pow(wrench_rh->wrench.force.z,2));

            if (wrench_lf_norm > grf_threshold_) stance_lf = true; else stance_lf = false;
            if (wrench_rf_norm > grf_threshold_) stance_rf = true; else stance_rf = false;
            if (wrench_lh_norm > grf_threshold_) stance_lh = true; else stance_lh = false;
            if (wrench_rh_norm > grf_threshold_) stance_rh = true; else stance_rh = false;


            // publishing
			msg_.header.stamp = this->node_->get_clock()->now();

			msg_.stance_lf = stance_lf;
			msg_.stance_rf = stance_rf;
			msg_.stance_lh = stance_lh;
			msg_.stance_rh = stance_rh;

			pub_->publish(msg_);

		} // end callback


	private:

		std::shared_ptr<message_filters::Subscriber<Wrench>> wrench_lf_sub_;
		std::shared_ptr<message_filters::Subscriber<Wrench>> wrench_rf_sub_;
		std::shared_ptr<message_filters::Subscriber<Wrench>> wrench_lh_sub_;
		std::shared_ptr<message_filters::Subscriber<Wrench>> wrench_rh_sub_;
		std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
		rclcpp::Publisher<state_estimator_msgs::msg::ContactDetection>::SharedPtr pub_;

		state_estimator_msgs::msg::ContactDetection msg_;

		bool stance_lf;
		bool stance_rf;
		bool stance_lh;
		bool stance_rh;
		double wrench_lf_norm;
		double wrench_rf_norm;
		double wrench_lh_norm;
		double wrench_rh_norm;
        double grf_threshold_;



	}; // end class ContactDetectionPlugin

} //end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ContactDetectionPlugin, state_estimator_plugins::PluginBase)
