#include "state_estimator/Models/sensor_fusion.hpp"
#include "state_estimator/plugin.hpp"

// ROS 2 message_filters
#include <message_filters/time_synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

#include <sensor_msgs/msg/imu.hpp>					 // read acceleration from imu
#include "state_estimator_msgs/msg/leg_odometry.hpp" // read base velocity from leg odometry
#include "state_estimator_msgs/msg/attitude.hpp"	 // read orientation from attitude estimation
#include <nav_msgs/msg/odometry.hpp>				 // read position from lidar odometry

#include <iit/commons/geometry/rotations.h>

namespace state_estimator_plugins
{

	typedef message_filters::sync_policies::ApproximateTime<
		sensor_msgs::msg::Imu,
		state_estimator_msgs::msg::Attitude,
		state_estimator_msgs::msg::LegOdometry>
		ApproximateTimePolicy;

	typedef message_filters::sync_policies::ExactTime<
		sensor_msgs::msg::Imu,
		state_estimator_msgs::msg::Attitude,
		state_estimator_msgs::msg::LegOdometry>
		ExactTimePolicy;

#define MySyncPolicy ApproximateTimePolicy

	class SensorFusionPlugin : public PluginBase
	{
	public:
		SensorFusionPlugin() : sensor_fusion_(nullptr),
							   imu_sub_(nullptr),
							   attitude_sub_(nullptr),
							   leg_odom_sub_(nullptr),
							   pub_(nullptr),
							   sync_(nullptr)
		{
		}

		~SensorFusionPlugin()
		{
			if (sensor_fusion_ != nullptr)
				delete (sensor_fusion_);
			if (imu_sub_ != nullptr)
				delete (imu_sub_);
			if (attitude_sub_ != nullptr)
				delete (attitude_sub_);
			if (leg_odom_sub_ != nullptr)
				delete (leg_odom_sub_);
			if (pub_ != nullptr)
				delete (pub_);
			if (sync_ != nullptr)
				delete (sync_);
		}

		std::string getName() override { return std::string("SensorFusion"); }
		std::string getDescription() override { return std::string("Sensor Fusion Plugin"); }

		void initialize_() override
		{
			t0 = 0.0;
			xhat_estimated << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

			// Load config parameters (ROS 2 parameter API)
			auto nh = this->node_;

			std::vector<double> P_vec, Q_vec, R_vec;
			nh->declare_parameter("sensor_fusion_plugin.P", std::vector<double>(36, 0.0));
			nh->declare_parameter("sensor_fusion_plugin.Q", std::vector<double>(36, 0.0));
			nh->declare_parameter("sensor_fusion_plugin.R", std::vector<double>(9, 0.0));
			
			P_vec = nh->get_parameter("sensor_fusion_plugin.P").as_double_array();
			Q_vec = nh->get_parameter("sensor_fusion_plugin.Q").as_double_array();
			R_vec = nh->get_parameter("sensor_fusion_plugin.R").as_double_array();

			if (P_vec.size() != 36 || Q_vec.size() != 36 || R_vec.size() != 9)
			{
				RCLCPP_ERROR(nh->get_logger(), "P0, Q0, or R0 parameters have incorrect size. Each should have 36 elements.");
				return;
			}

			P = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P_vec.data());
			Q = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q_vec.data());
			R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_vec.data());

			sensor_fusion_ = new state_estimator::KFSensorFusion(t0, xhat_estimated, P, Q, R, false, false);

			nh->declare_parameter("sensor_fusion_plugin.base_R_imu", std::vector<double>(9, 0.0));
			std::vector<double> base_R_imu_vec = nh->get_parameter("sensor_fusion_plugin.base_R_imu").as_double_array();

			if (base_R_imu_vec.size() == 9)
			{
				base_R_imu_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(base_R_imu_vec.data());
				RCLCPP_INFO(nh->get_logger(),"Loaded base_R_imu:\n" << base_R_imu_);
			}
			else
			{
				RCLCPP_WARN(nh->get_logger(), "base_R_imu is not the correct size (should be 9). Using identity.");
				base_R_imu_ = Eigen::Matrix3d::Identity();
			}

			//Declare parameters
			const std::string imu_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.imu_topic", "/sensors/imu");
			const std::string attitude_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.attitude_topic", "/sensors/imu");
			const std::string leg_odom_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.leg_odometry_topic", "/sensors/leg_odometry");
			const std::string pub_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.pub_topic", "/sensors/odometry");

			// QoS profile for sensor data
			auto sensor_qos = rclcpp::SensorDataQoS();

			imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(nh, imu_topic, sensor_qos);
			attitude_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>>(nh, attitude_topic, sensor_qos);
			leg_odom_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::LegOdometry>>(nh, leg_odom_topic, sensor_qos);

			//Synchronizer
			sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(100), *imu_sub_, *attitude_sub_, *leg_odom_sub_);
			sync_->registerCallback(std::bind(&SensorFusionPlugin::callback_proprioception, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

			// Publisher
			pub_ = nh->create_publisher<nav_msgs::msg::Odometry>(pub_topic, sensor_qos);

			RCLCPP_INFO(nh->get_logger(), "SensorFusionPlugin initialized");
		}

		void shutdown_() override {}
		void pause_() override {}
		void resume_() override {}
		void reset_() override {}

		void callback_proprioception(
			const sensor_msgs::msg::Imu::ConstPtr &imu,
			const state_estimator_msgs::msg::Attitude::ConstPtr &attitude,
			const state_estimator_msgs::msg::LegOdometry::ConstPtr &leg_odom)
		{
			// Reading imu
			Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

			// Reading attitude estimation
			omega << attitude->angular_velocity[0], attitude->angular_velocity[1], attitude->angular_velocity[2];
			quat_est.w() = attitude->quaternion[0];
			quat_est.vec() << attitude->quaternion[1], attitude->quaternion[2], attitude->quaternion[3];
			rpy = iit::commons::quatToRPY(quat_est);
			w_R_b = iit::commons::quatToRotMat(quat_est).transpose();

			// Reading leg odometry
			v_b << leg_odom->base_velocity[0], leg_odom->base_velocity[1], leg_odom->base_velocity[2];
			computeLinPosVel(acc, w_R_b, v_b);
		}

		void computeLinPosVel(Eigen::Vector3d &acc, Eigen::Matrix3d &w_R_b, Eigen::Vector3d &v_b)
		{
			if (begin)
			{
				time_begin_ = this->node_->get_clock()->now().seconds();
				w_R_wb = w_R_b;
				begin = false;
			}

			time_ = this->node_->get_clock()->now().seconds() - time_begin_;

			// reading acceleration from imu
			Eigen::Vector3d f_b = base_R_imu_ * acc;

			// input u = w_R_b*f_b - gravity
			Eigen::Vector3d gravity;
			gravity << 0.0, 0.0, -9.81;
			Eigen::Vector3d u = w_R_b * f_b + gravity;
			// prediction
			sensor_fusion_->predict(time_, u);

			// reading leg odometry
			Eigen::Vector3d w_v_b = w_R_b * v_b;

			Eigen::Vector3d z_proprio;
			z_proprio << w_v_b;

			// // correction
			sensor_fusion_->update(time_, z_proprio);
			xhat_estimated = sensor_fusion_->getX();

			// publish
			msg_.header.stamp = this->node_->get_clock()->now();

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
		state_estimator::KFSensorFusion *sensor_fusion_;

		double t0;
		Eigen::Matrix<double, 6, 1> xhat_estimated;
		Eigen::Matrix<double, 6, 6> P;
		Eigen::Matrix<double, 6, 6> Q;
		Eigen::Matrix<double, 3, 3> R;

		std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;
		std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>> attitude_sub_;
		std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::LegOdometry>> leg_odom_sub_;

		std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

		nav_msgs::msg::Odometry msg_;

		double time_{};
		bool begin{true};
		double time_begin_;
		Eigen::Vector3d v_b;
		Eigen::Quaterniond quat_est;
		Eigen::Vector3d omega;
		double yaw;
		Eigen::Vector3d rpy;
		Eigen::Matrix3d w_R_b;
		Eigen::Matrix3d base_R_imu_;

		Eigen::Matrix3d w_R_wb; // world frame

	}; // end class SensorFusionPlugin

} // end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::SensorFusionPlugin, state_estimator_plugins::PluginBase)
