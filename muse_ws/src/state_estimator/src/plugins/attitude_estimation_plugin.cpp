#include "state_estimator/plugin.hpp"
#include "state_estimator/Models/attitude_bias_NLO.hpp"
#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "iit/commons/geometry/rotations.h"

#include "state_estimator_msgs/msg/attitude.hpp"

#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>                     // read acceleration and angular velocity from imu
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>


namespace state_estimator_plugins 
{
	class AttitudeEstimationPlugin : public PluginBase 
	{
	public:

		AttitudeEstimationPlugin() = default;

		~AttitudeEstimationPlugin() override 
		{
			// smart pointers handle cleanup
			if (attitude_ != nullptr) {
				delete attitude_;
				attitude_ = nullptr;
			}
		}

		std::string getName() override { return std::string("AttitudeEstimation"); }
		std::string getDescription() override { return std::string("Attitude Estimation Plugin"); }

		void initialize_() override 
		{
			t0 = 0.0;
			xhat_estimated << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			xhat_estimated.head<4>() = xhat_estimated.head<4>() / xhat_estimated.head<4>().norm();

			// Load config parameters (ROS 2 parameter API)
			auto nh = this->node_;
			std::string imu_topic = nh->declare_parameter<std::string>("attitude_estimation_plugin.imu_topic", "/sensors/imu");
			std::string pub_topic = nh->declare_parameter<std::string>("attitude_estimation_plugin.pub_topic", "attitude");
			double ki_param = nh->declare_parameter<double>("attitude_estimation_plugin.ki", 0.02);
			double kp_param = nh->declare_parameter<double>("attitude_estimation_plugin.kp", 10.0);
			std::vector<double> base_R_imu_vec = nh->declare_parameter<std::vector<double>>("attitude_estimation_plugin.base_R_imu", {});
			std::vector<double> north_vec = nh->declare_parameter<std::vector<double>>("attitude_estimation_plugin.north_vector", {});
			std::vector<double> gravity_vec = nh->declare_parameter<std::vector<double>>("attitude_estimation_plugin.gravity_vector", {});
			std::vector<double> P_vec = nh->declare_parameter<std::vector<double>>("attitude_estimation_plugin.P", {});
			std::vector<double> Q_vec = nh->declare_parameter<std::vector<double>>("attitude_estimation_plugin.Q", {});
			std::vector<double> R_vec = nh->declare_parameter<std::vector<double>>("attitude_estimation_plugin.R", {});

			// Set gains
			ki = ki_param;
			kp = kp_param;

			// Convert base_R_imu
			if (base_R_imu_vec.size() == 9) {
			    b_R_imu = Eigen::Map<Eigen::Matrix3d>(base_R_imu_vec.data());
			} else {
				RCLCPP_WARN(nh->get_logger(), "Invalid or missing base_R_imu config — using identity.");
			    b_R_imu.setIdentity();
			}

			// Convert north_vector
			if (north_vec.size() == 3) {
			    m_n = Eigen::Map<Eigen::Vector3d>(north_vec.data());
			} else {
			    RCLCPP_WARN(nh->get_logger(), "Invalid or missing north_vector config — using default.");
			    m_n << 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0);
			}

			// Convert gravity_vector
			if (gravity_vec.size() == 3) {
			    f_n = Eigen::Map<Eigen::Vector3d>(gravity_vec.data());
			} else {
			    RCLCPP_WARN(nh->get_logger(), "Invalid or missing gravity_vector config — using default.");
			    f_n << 0.0, 0.0, 9.81;
			}
			
			// Matrices P0, Q, R (6x6) from parameter vectors; use safe defaults if sizes mismatch
			Eigen::Matrix<double, 6, 6> P0_local = Eigen::Matrix<double, 6, 6>::Identity();
			Eigen::Matrix<double, 6, 6> Q_local = Eigen::Matrix<double, 6, 6>::Identity();
			Eigen::Matrix<double, 6, 6> R_local = Eigen::Matrix<double, 6, 6>::Identity();
			if (P_vec.size() == 36) {
				P0_local = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P_vec.data());
			} else if (!P_vec.empty()) {
				RCLCPP_WARN(nh->get_logger(), "Parameter P should have 36 elements; using Identity.");
			}
			if (Q_vec.size() == 36) {
				Q_local = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q_vec.data());
			} else if (!Q_vec.empty()) {
				RCLCPP_WARN(nh->get_logger(), "Parameter Q should have 36 elements; using Identity.");
			}
			if (R_vec.size() == 36) {
				R_local = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(R_vec.data());
			} else if (!R_vec.empty()) {
				RCLCPP_WARN(nh->get_logger(), "Parameter R should have 36 elements; using Identity.");
			}

			attitude_ = new state_estimator::AttitudeBiasXKF(t0, xhat_estimated, P0_local, Q_local, R_local, f_n, m_n, ki, kp);

			// Set up ROS 2 interfaces
			using std::placeholders::_1;
			imu_sub_ = nh->create_subscription<sensor_msgs::msg::Imu>(
				imu_topic,
				rclcpp::SensorDataQoS(),
				std::bind(&AttitudeEstimationPlugin::callback_imu, this, _1));
			pub_ = nh->create_publisher<state_estimator_msgs::msg::Attitude>(pub_topic, rclcpp::QoS(10));

			RCLCPP_INFO(nh->get_logger(), "AttitudeEstimationPlugin initialized with topic '%s'", imu_topic.c_str());
  				
		}


		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback_imu(
			const sensor_msgs::msg::Imu::ConstSharedPtr imu)
		{
			Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
	        Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);	
			computeAttitude(omega, acc);		
		}

		void computeAttitude(Eigen::Vector3d &omega, Eigen::Vector3d &acc)
		{
			/* to take the time from a rosbag:
			$ roscore
			$ rosparam set use_sim_time true
			$ rosbag play name_of_the_bag.bag --clock
			*/
	
			if (begin)
			{
				time_begin_ = this->node_->get_clock()->now().seconds();
				begin = false;
			}		

			time_ = this->node_->get_clock()->now().seconds() - time_begin_;

	      	quat_est.w() = xhat_estimated(0);
	      	quat_est.vec() << xhat_estimated(1), xhat_estimated(2), xhat_estimated(3);

	      	f_b = b_R_imu * acc;
	      	m_b = iit::commons::quatToRotMat(quat_est) * m_n;

	      	z << f_b, m_b; 
	
			attitude_->update(time_, b_R_imu * omega, z);

	      	xhat_estimated = attitude_->getX();
			// sync quaternion from updated state
			quat_est.w() = xhat_estimated(0);
			quat_est.vec() << xhat_estimated(1), xhat_estimated(2), xhat_estimated(3);
			
			xdot = attitude_->calc_f(time_, xhat_estimated, b_R_imu * omega);
			quat_dot.w() = xdot(0);
			quat_dot.vec() << xdot(1), xdot(2), xdot(3);
			omega_filt = iit::commons::quatToOmega(quat_est, quat_dot);

			// // publishing
			msg_.header.stamp = this->node_->get_clock()->now();

			msg_.quaternion[0] = quat_est.w();
			msg_.quaternion[1] = quat_est.x();
			msg_.quaternion[2] = quat_est.y();
			msg_.quaternion[3] = quat_est.z();

			euler_radians = iit::commons::quatToRPY(quat_est);
	      	euler_degrees = euler_radians * (180.0 / M_PI);

			msg_.roll_deg  = euler_degrees(0);
			msg_.pitch_deg = euler_degrees(1);
			msg_.yaw_deg   = euler_degrees(2);

			msg_.angular_velocity[0] = omega_filt(0);
			msg_.angular_velocity[1] = omega_filt(1);
			msg_.angular_velocity[2] = omega_filt(2);

			pub_->publish(msg_);

		} // end computeAttitude


	private:

		state_estimator::AttitudeBiasXKF *attitude_{nullptr};
		Eigen::Matrix<double, 7, 1> xhat_estimated;
		Eigen::Matrix<double, 3, 1> f_n; 
		Eigen::Matrix<double, 3, 1> m_n; 
    	Eigen::Matrix3d b_R_imu;
		double ki; 
		double kp; 
		double t0; 
	
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
		rclcpp::Publisher<state_estimator_msgs::msg::Attitude>::SharedPtr pub_;

    	Eigen::Quaterniond quat_est;
    	Eigen::Vector3d f_b;
    	Eigen::Vector3d m_b; 
	    Eigen::Matrix<double, 6, 1> z;
	    Eigen::Matrix<double, 7, 1> xdot;
    	Eigen::Quaterniond quat_dot;
    	Eigen::Vector3d omega_filt;
    	Eigen::Vector3d euler_radians;
    	Eigen::Vector3d euler_degrees;

		state_estimator_msgs::msg::Attitude msg_;
	
		double time_{};
		bool begin{true};
		double time_begin_;

	}; // end class AttitudeEstimationPlugin

} // end namespace state_estimator_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::AttitudeEstimationPlugin, state_estimator_plugins::PluginBase)
