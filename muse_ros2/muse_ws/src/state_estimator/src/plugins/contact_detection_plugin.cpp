#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>

#include "state_estimator_msgs/msg/contact_detection.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cmath>
#include <memory>
#include <functional>


namespace state_estimator_plugins
{

typedef message_filters::sync_policies::ApproximateTime
<
	geometry_msgs::msg::WrenchStamped,
	geometry_msgs::msg::WrenchStamped,
	geometry_msgs::msg::WrenchStamped,
	geometry_msgs::msg::WrenchStamped
> 
ApproximateTimePolicy;

typedef message_filters::sync_policies::ExactTime
<
	geometry_msgs::msg::WrenchStamped,
	geometry_msgs::msg::WrenchStamped,
	geometry_msgs::msg::WrenchStamped,
	geometry_msgs::msg::WrenchStamped
> 
ExactTimePolicy;

#define MySyncPolicy ApproximateTimePolicy

	class ContactDetectionPlugin : public PluginBase
	{
	public:
		ContactDetectionPlugin(): 
			wrench_lf_sub_(nullptr),
			wrench_rf_sub_(nullptr),
			wrench_lh_sub_(nullptr),
			wrench_rh_sub_(nullptr),
			pub_(nullptr), 
			sync_(nullptr) 
		{ } 
	
		~ContactDetectionPlugin() 
		{
			// Smart pointers will be automatically destroyed
		}

		std::string getName() override { return std::string("ContactDetection"); }
		std::string getDescription() override { return std::string("Contact Detection Plugin"); }

		void initialize_() override {

            // Load parameters from YAML
            // LF: left front leg, RF: right front leg, LH: left hind leg, RH: right hind leg
            std::string lf_topic, rf_topic, lh_topic, rh_topic, pub_topic;
            
            node_->declare_parameter("contact_detection_plugin.wrench_lf_topic", "/state_estimator/contact_force_lf_foot");
            node_->declare_parameter("contact_detection_plugin.wrench_rf_topic", "/state_estimator/contact_force_rf_foot");
            node_->declare_parameter("contact_detection_plugin.wrench_lh_topic", "/state_estimator/contact_force_lh_foot");
            node_->declare_parameter("contact_detection_plugin.wrench_rh_topic", "/state_estimator/contact_force_rh_foot");
            node_->declare_parameter("contact_detection_plugin.pub_topic", "/state_estimator/contact_detection");
            node_->declare_parameter("contact_detection_plugin.grf_threshold", 15.0);
            
            lf_topic = node_->get_parameter("contact_detection_plugin.wrench_lf_topic").as_string();
            rf_topic = node_->get_parameter("contact_detection_plugin.wrench_rf_topic").as_string();
            lh_topic = node_->get_parameter("contact_detection_plugin.wrench_lh_topic").as_string();
            rh_topic = node_->get_parameter("contact_detection_plugin.wrench_rh_topic").as_string();
            pub_topic = node_->get_parameter("contact_detection_plugin.pub_topic").as_string();
            grf_threshold_ = node_->get_parameter("contact_detection_plugin.grf_threshold").as_double();

            RCLCPP_INFO_STREAM(node_->get_logger(), "ContactDetectionPlugin loaded with GRF threshold: " << grf_threshold_);

            // Set up subscribers
            wrench_lf_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>>(node_, lf_topic);
            wrench_rf_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>>(node_, rf_topic);
            wrench_lh_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>>(node_, lh_topic);
            wrench_rh_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>>(node_, rh_topic);

            // Synchronizer
            sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(250), *wrench_lf_sub_, *wrench_rf_sub_, *wrench_lh_sub_, *wrench_rh_sub_);
            sync_->registerCallback(std::bind(&ContactDetectionPlugin::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

            // Publisher
            pub_ = node_->create_publisher<state_estimator_msgs::msg::ContactDetection>(pub_topic, 250);

        }
		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback
		(
			const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& wrench_lf,
			const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& wrench_rf,
			const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& wrench_lh,
			const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& wrench_rh
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
            msg_.header.stamp = node_->get_clock()->now();

			msg_.stance_lf = stance_lf;
			msg_.stance_rf = stance_rf;
			msg_.stance_lh = stance_lh;
			msg_.stance_rh = stance_rh;

			pub_->publish(msg_);

		} // end callback


	private:
	
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>> wrench_lf_sub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>> wrench_rf_sub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>> wrench_lh_sub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::WrenchStamped>> wrench_rh_sub_;
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
