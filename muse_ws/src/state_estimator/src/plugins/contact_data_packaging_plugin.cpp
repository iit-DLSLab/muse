#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>

#include "state_estimator_msgs/msg/contact_detection.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <message_filters/time_synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

namespace state_estimator_plugins
{
using boolean_msg = std_msgs::msg::Bool;
using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<boolean_msg, boolean_msg, boolean_msg, boolean_msg>;
using ExactTimePolicy = message_filters::sync_policies::ExactTime<boolean_msg, boolean_msg, boolean_msg, boolean_msg>;

#define MySyncPolicy ApproximateTimePolicy

    class ContactDataPackagingPlugin : public PluginBase
    {
        public: ContactDataPackagingPlugin() = default;

        ~ContactDataPackagingPlugin() override = default;

        std::string getName() override { return std::string("ContactDataPackagingPlugin"); }
        std::string getDescription() override { return std::string("This plugin reads contact sensor data and packages them into a ContactDetection method for the other plugins to use");}

        void initialize_() override
        {
            auto node = this->node_;
            if(!node) { throw std::runtime_error("ContactDataPackagingPlugin: node_ is null"); }

            //ROS2 Parameters
			const std::string lf_topic = node->declare_parameter<std::string>("contact_data_packaging_plugin.contact_lf_topic", "/contact_lf");
			const std::string rf_topic = node->declare_parameter<std::string>("contact_data_packaging_plugin.contact_rf_topic", "/contact_rf");
			const std::string lh_topic = node->declare_parameter<std::string>("contact_data_packaging_plugin.contact_lh_topic", "/contact_lh");
			const std::string rh_topic = node->declare_parameter<std::string>("contact_data_packaging_plugin.contact_rh_topic", "/contact_rh");
            const std::string pub_topic = node->declare_parameter<std::string>("contact_data_packaging_plugin.pub_topic", "contact_detection");

            RCLCPP_INFO(node->get_logger(), "ContactDataPackaginPlugin loaded");

            //QoS profile for sensor data
            auto sensor_qos = rclcpp::SensorDataQoS();

            //Contact data subscribers
			contact_lf_sub_ = std::make_shared<message_filters::Subscriber<boolean_msg>>(node, lf_topic, sensor_qos);
			contact_rf_sub_ = std::make_shared<message_filters::Subscriber<boolean_msg>>(node, rf_topic, sensor_qos);
			contact_lh_sub_ = std::make_shared<message_filters::Subscriber<boolean_msg>>(node, lh_topic, sensor_qos);
			contact_rh_sub_ = std::make_shared<message_filters::Subscriber<boolean_msg>>(node, rh_topic, sensor_qos);

			// Publisher
			pub_ = node->create_publisher<state_estimator_msgs::msg::ContactDetection>(pub_topic, sensor_qos);

            // Synchronizer
			sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(250), *contact_lf_sub_, *contact_rf_sub_, *contact_lh_sub_, *contact_rh_sub_);
			sync_->registerCallback(std::bind(&ContactDataPackagingPlugin::callback, this,
											  std::placeholders::_1,
											  std::placeholders::_2,
											  std::placeholders::_3,
											  std::placeholders::_4));

        }

    	void shutdown_() override {}
		void pause_() override {}
		void resume_() override {}
		void reset_() override {}

		void callback(
			const boolean_msg::ConstSharedPtr contact_lf,
			const boolean_msg::ConstSharedPtr contact_rf,
			const boolean_msg::ConstSharedPtr contact_lh,
			const boolean_msg::ConstSharedPtr contact_rh)
		{

			// publishing
			msg_.header.stamp = this->node_->get_clock()->now();

            //simply set the contact state, aggregating the incoming 4 seperate sensor topics into 1 message
			msg_.stance_lf = contact_lf->data;
			msg_.stance_rf = contact_rf->data;
			msg_.stance_lh = contact_lh->data;
			msg_.stance_rh = contact_rh->data;

			pub_->publish(msg_);

		} // end callback

	private:
		std::shared_ptr<message_filters::Subscriber<boolean_msg>> contact_lf_sub_;
		std::shared_ptr<message_filters::Subscriber<boolean_msg>> contact_rf_sub_;
		std::shared_ptr<message_filters::Subscriber<boolean_msg>> contact_lh_sub_;
		std::shared_ptr<message_filters::Subscriber<boolean_msg>> contact_rh_sub_;
		std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
		rclcpp::Publisher<state_estimator_msgs::msg::ContactDetection>::SharedPtr pub_;

		state_estimator_msgs::msg::ContactDetection msg_;

		bool stance_lf;
		bool stance_rf;
		bool stance_lh;
		bool stance_rh;

    };
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ContactDataPackagingPlugin, state_estimator_plugins::PluginBase)