#include "state_estimator/Models/sensor_fusion.hpp"
#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/imu.hpp>                        	// read acceleration from imu
#include "state_estimator_msgs/msg/leg_odometry.hpp"      	    // read base velocity from leg odometry
#include "state_estimator_msgs/msg/attitude.hpp"          	// read orientation from attitude estimation
#include <nav_msgs/msg/odometry.hpp>							// read position from lidar odometry

#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace state_estimator_plugins
{

typedef message_filters::sync_policies::ApproximateTime
<
	sensor_msgs::msg::Imu,
	state_estimator_msgs::msg::Attitude,
	state_estimator_msgs::msg::LegOdometry 
> 
ApproximateTimePolicy;

typedef message_filters::sync_policies::ExactTime
<
	sensor_msgs::msg::Imu,
	state_estimator_msgs::msg::Attitude,
	state_estimator_msgs::msg::LegOdometry
> 
ExactTimePolicy;

#define MySyncPolicy ApproximateTimePolicy

	class SensorFusionPlugin : public PluginBase  
	{
	public:
		SensorFusionPlugin():
			sensor_fusion_(nullptr),
			imu_sub_(nullptr),
			attitude_sub_(nullptr),
			leg_odom_sub_(nullptr),
			pub_(nullptr),
			sync_(nullptr)
		{ }

        ~SensorFusionPlugin()
        {
            // Smart pointers will be automatically destroyed
        }		std::string getName() override { return std::string("SensorFusion"); }
		std::string getDescription() override { return std::string("Sensor Fusion Plugin"); }

		void initialize_() override 
		{
        	t0 = 0.0;
        	xhat_estimated << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        	std::vector<double> P_vec, Q_vec, R_vec;
        	
        	RCLCPP_INFO(node_->get_logger(), "Initializing SensorFusionPlugin...");
        	
        	node_->declare_parameter("sensor_fusion_plugin.P", std::vector<double>(36, 0.0));
        	node_->declare_parameter("sensor_fusion_plugin.Q", std::vector<double>(36, 0.0));
        	node_->declare_parameter("sensor_fusion_plugin.R", std::vector<double>(9, 0.0));
        	
        	try {
        		P_vec = node_->get_parameter("sensor_fusion_plugin.P").get_value<std::vector<double>>();
        		RCLCPP_INFO_STREAM(node_->get_logger(), "P matrix size: " << P_vec.size());
        		
        		Q_vec = node_->get_parameter("sensor_fusion_plugin.Q").get_value<std::vector<double>>();
        		RCLCPP_INFO_STREAM(node_->get_logger(), "Q matrix size: " << Q_vec.size());
        		
        		R_vec = node_->get_parameter("sensor_fusion_plugin.R").get_value<std::vector<double>>();
        		RCLCPP_INFO_STREAM(node_->get_logger(), "R matrix size: " << R_vec.size());
        	} catch (const std::exception& e) {
        		RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get matrix parameters: " << e.what());
        		return;
        	}

        	if (P_vec.size() != 36 || Q_vec.size() != 36 || R_vec.size() != 9) {
        		RCLCPP_ERROR(node_->get_logger(), "P0, Q0, or R0 parameters have incorrect size. Each should have 36 elements.");
        		return;
        	}

        	P = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P_vec.data());
        	Q = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q_vec.data());
        	R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_vec.data());

            sensor_fusion_ = std::make_unique<state_estimator::KFSensorFusion>(t0, xhat_estimated, P, Q, R, false, false);            std::vector<double> base_R_imu_vec;
            node_->declare_parameter("sensor_fusion_plugin.base_R_imu", std::vector<double>(9, 0.0));
            try {
                base_R_imu_vec = node_->get_parameter("sensor_fusion_plugin.base_R_imu").get_value<std::vector<double>>();
                RCLCPP_INFO_STREAM(node_->get_logger(), "base_R_imu loaded, size: " << base_R_imu_vec.size());
            } catch (const std::exception& e) {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get base_R_imu parameter: " << e.what());
                base_R_imu_vec = std::vector<double>(9, 0.0);
            }

            if (base_R_imu_vec.size() == 9) {
                base_R_imu_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(base_R_imu_vec.data());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded base_R_imu:\n" << base_R_imu_);
            } else {
                RCLCPP_WARN(node_->get_logger(), "base_R_imu is not the correct size (should be 9). Using identity.");
                base_R_imu_ = Eigen::Matrix3d::Identity();
            }

        	std::string imu_topic, attitude_topic, leg_odom_topic, pub_topic;
        	node_->declare_parameter("sensor_fusion_plugin.imu_topic", "/sensors/imu");
        	node_->declare_parameter("sensor_fusion_plugin.attitude_topic", "/attitude");
        	node_->declare_parameter("sensor_fusion_plugin.leg_odometry_topic", "/state_estimator/leg_odometry");
        	node_->declare_parameter("sensor_fusion_plugin.pub_topic", "sensor_fusion");
        	
        	imu_topic = node_->get_parameter("sensor_fusion_plugin.imu_topic").as_string();
        	attitude_topic = node_->get_parameter("sensor_fusion_plugin.attitude_topic").as_string();
        	leg_odom_topic = node_->get_parameter("sensor_fusion_plugin.leg_odometry_topic").as_string();
        	pub_topic = node_->get_parameter("sensor_fusion_plugin.pub_topic").as_string();

        	imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(node_, imu_topic);
        	attitude_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>>(node_, attitude_topic);
        	leg_odom_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::LegOdometry>>(node_, leg_odom_topic);

        	sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(100), *imu_sub_, *attitude_sub_, *leg_odom_sub_);
        	sync_->registerCallback(std::bind(&SensorFusionPlugin::callback_proprioception, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        	pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(pub_topic, 250);

        	RCLCPP_INFO(node_->get_logger(), "SensorFusionPlugin initialized");
		}

		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback_proprioception
		(
			const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
			const state_estimator_msgs::msg::Attitude::ConstSharedPtr& attitude,
			const state_estimator_msgs::msg::LegOdometry::ConstSharedPtr& leg_odom		
		)
		{
			// Reading imu
			Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

			// Reading attitude estimation	
            omega << attitude->angular_velocity[0], attitude->angular_velocity[1], attitude->angular_velocity[2];		
			quat_est.w() = attitude->quaternion[0];
			quat_est.vec() << attitude->quaternion[1], attitude->quaternion[2], attitude->quaternion[3];
			// rpy = iit::commons::quatToRPY(quat_est);
            w_R_b = quat_est.toRotationMatrix().transpose();

			// Reading leg odometry
			v_b << leg_odom->base_velocity[0], leg_odom->base_velocity[1], leg_odom->base_velocity[2];
			computeLinPosVel(acc,w_R_b,v_b);
		}

		void computeLinPosVel(Eigen::Vector3d &acc, Eigen::Matrix3d &w_R_b, Eigen::Vector3d &v_b)
		{ 
			if (begin)
			{
				time_begin_ = node_->get_clock()->now();
				w_R_wb = w_R_b;
				begin = false;
			}		

			auto current_time = node_->get_clock()->now();
			time_ = (current_time - time_begin_).seconds();

			// reading acceleration from imu
 			Eigen::Vector3d f_b = base_R_imu_*acc;

			// input u = w_R_b*f_b - gravity
			Eigen::Vector3d gravity; gravity << 0.0, 0.0, -9.81;
			Eigen::Vector3d u = w_R_b*f_b + gravity;
			// prediction
			sensor_fusion_->predict(time_,u);

			// reading leg odometry
			Eigen::Vector3d w_v_b = w_R_b*v_b;

			Eigen::Vector3d z_proprio;
			z_proprio << w_v_b;

			// // correction
			sensor_fusion_->update(time_,z_proprio);
			xhat_estimated = sensor_fusion_->getX();

			// publish
			msg_.header.stamp = current_time;
			
			msg_.pose.pose.orientation.w = quat_est.w();
			msg_.pose.pose.orientation.x = quat_est.x();
			msg_.pose.pose.orientation.y = quat_est.y();
			msg_.pose.pose.orientation.z = quat_est.z();	

			msg_.pose.pose.position.x = xhat_estimated(0);
			msg_.pose.pose.position.y = xhat_estimated(1);
			msg_.pose.pose.position.z = xhat_estimated(2);	

			msg_.twist.twist.linear.x = xhat_estimated(3);
			msg_.twist.twist.linear.y = xhat_estimated(4);
			msg_.twist.twist.linear.z = xhat_estimated(5);

            msg_.twist.twist.angular.x = omega[0];
            msg_.twist.twist.angular.y = omega[1];
            msg_.twist.twist.angular.z = omega[2];

			pub_->publish(msg_);

		} // end computeLinPosVel


	private:

		std::unique_ptr<state_estimator::KFSensorFusion> sensor_fusion_;

    	double t0;
    	Eigen::Matrix<double,6,1> xhat_estimated;
    	Eigen::Matrix<double,6,6> P;
    	Eigen::Matrix<double,6,6> Q;
    	Eigen::Matrix<double,3,3> R;

		std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;
		std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>> attitude_sub_;
		std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::LegOdometry>> leg_odom_sub_;
	
		std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

		nav_msgs::msg::Odometry msg_;

		double time_{};
		bool begin{true};
		rclcpp::Time time_begin_;
        Eigen::Vector3d v_b;
		Eigen::Quaterniond quat_est;
        Eigen::Vector3d omega;
		double yaw;
		Eigen::Vector3d rpy;
		Eigen::Matrix3d w_R_b; 
        Eigen::Matrix3d base_R_imu_;

		Eigen::Matrix3d w_R_wb; // world frame

	};	// end class SensorFusionPlugin

} 	// end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::SensorFusionPlugin, state_estimator_plugins::PluginBase)



