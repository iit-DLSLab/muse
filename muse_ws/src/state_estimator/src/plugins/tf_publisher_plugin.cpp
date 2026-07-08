#include "state_estimator/plugin.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

namespace state_estimator_plugins
{

	class TfPublisherPlugin : public PluginBase
	{
	public:
		TfPublisherPlugin():
			odom_sub_(nullptr),
			tf_broadcaster_(nullptr)
		{ }

		~TfPublisherPlugin()
		{
			if (odom_sub_!=nullptr) delete(odom_sub_);
			if (tf_broadcaster_!=nullptr) delete(tf_broadcaster_);
		}

		std::string getName() override { return std::string("TfPublisher"); }
		std::string getDescription() override { return std::string("TF Publisher Plugin"); }

		void initialize_() override
		{
			nh_.param<std::string>("tf_publisher_plugin/odom_topic", odom_topic_, "/state_estimator/sensor_fusion");
			nh_.param<std::string>("tf_publisher_plugin/parent_frame", parent_frame_, "world");
			nh_.param<std::string>("tf_publisher_plugin/child_frame", child_frame_, "base");
			nh_.param<bool>("tf_publisher_plugin/use_odom_frame_ids", use_odom_frame_ids_, false);

			tf_broadcaster_ = new tf2_ros::TransformBroadcaster();
			odom_sub_ = new ros::Subscriber(nh_.subscribe(odom_topic_, 250, &TfPublisherPlugin::callback, this));

			ROS_INFO_STREAM(
				"TfPublisherPlugin initialized: broadcasting "
				<< parent_frame_ << " -> " << child_frame_
				<< " from " << odom_topic_
			);
		}

		void shutdown_() override
		{
			if (odom_sub_!=nullptr) odom_sub_->shutdown();
		}

		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

	private:
		void callback(const nav_msgs::Odometry::ConstPtr& odom)
		{
			if (isPaused()) return;

			geometry_msgs::TransformStamped transform;
			transform.header.stamp = odom->header.stamp.isZero() ? ros::Time::now() : odom->header.stamp;
			transform.header.frame_id = parent_frame_;
			transform.child_frame_id = child_frame_;

			if (use_odom_frame_ids_)
			{
				if (!odom->header.frame_id.empty()) transform.header.frame_id = odom->header.frame_id;
				if (!odom->child_frame_id.empty()) transform.child_frame_id = odom->child_frame_id;
			}

			transform.transform.translation.x = odom->pose.pose.position.x;
			transform.transform.translation.y = odom->pose.pose.position.y;
			transform.transform.translation.z = odom->pose.pose.position.z;
			transform.transform.rotation = odom->pose.pose.orientation;

			tf_broadcaster_->sendTransform(transform);
		}

		ros::Subscriber *odom_sub_;
		tf2_ros::TransformBroadcaster *tf_broadcaster_;

		std::string odom_topic_;
		std::string parent_frame_;
		std::string child_frame_;
		bool use_odom_frame_ids_{false};
	};

} 	// end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::TfPublisherPlugin, state_estimator_plugins::PluginBase)
