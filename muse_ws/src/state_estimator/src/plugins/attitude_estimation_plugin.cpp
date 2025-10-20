#include "state_estimator/plugin.hpp"
#include "state_estimator/Models/attitude_bias_NLO.hpp"
#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "iit/commons/geometry/rotations.h"

#include "state_estimator_msgs/msg/attitude.h"
#include "state_estimator/lib.hpp"

#include <rosgraph_msgs/msg/clock.h>
#include <sensor_msgs/msg/imu.h>						  	// read acceleration and angular velocity from imu
#include <nav_msgs/msg/odometry.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "rosbag2_transport/reader_writer_factory.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>


namespace state_estimator_plugins 
{
	class AttitudeEstimationPlugin : public PluginBase 
	{
	public:

		AttitudeEstimationPlugin():
		attitude_(nullptr),
		imu_sub_(nullptr),
		pub_(nullptr)
		{ } 	

		~AttitudeEstimationPlugin() 
		{
			if (attitude_!=nullptr) delete(attitude_);
      		if (imu_sub_!=nullptr) delete(imu_sub_);
			if (pub_!=nullptr) delete(pub_);
		}

		std::string getName() override { return std::string("AttitudeEstimation"); }
		std::string getDescription() override { return std::string("Attitude Estimation Plugin"); }

		void initialize_() override 
		{
    		t0 = 0.0;
    		xhat_estimated << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    		xhat_estimated.head(4) = xhat_estimated.head(4) / xhat_estimated.head(4).norm();

    		// Load config parameters
    		std::string imu_topic, pub_topic;
    		std::vector<double> base_R_imu_vec, north_vec, gravity_vec;
			std::vector<double> P_vec, Q_vec, R_vec;
    		double ki_param, kp_param;

			rclcpp::Node::SharedPtr nh_ = this->node_;

    		nh_.param("attitude_estimation_plugin/imu_topic", imu_topic, std::string("/sensors/imu"));
    		nh_.param("attitude_estimation_plugin/pub_topic", pub_topic, std::string("attitude"));
    		nh_.param("attitude_estimation_plugin/ki", ki_param, 0.02);
    		nh_.param("attitude_estimation_plugin/kp", kp_param, 10.0);
    		nh_.param("attitude_estimation_plugin/base_R_imu", base_R_imu_vec, std::vector<double>());
    		nh_.param("attitude_estimation_plugin/north_vector", north_vec, std::vector<double>());
			nh_.param("attitude_estimation_plugin/gravity_vector", gravity_vec, std::vector<double>());
			nh_.param("attitude_estimation_plugin/P", P_vec, std::vector<double>());
    		nh_.param("attitude_estimation_plugin/Q", Q_vec, std::vector<double>());
    		nh_.param("attitude_estimation_plugin/R", R_vec, std::vector<double>());

    		// Set gains
    		ki = ki_param;
    		kp = kp_param;

    		// Convert base_R_imu
    		if (base_R_imu_vec.size() == 9) {
    		    b_R_imu = Eigen::Map<Eigen::Matrix3d>(base_R_imu_vec.data());
    		} else {
				RCLCPP_WARN(nh_.get_logger(), "Invalid or missing base_R_imu config — using identity.");
    		    b_R_imu.setIdentity();
    		}

    		// Convert north_vector
    		if (north_vec.size() == 3) {
    		    m_n = Eigen::Map<Eigen::Vector3d>(north_vec.data());
    		} else {
    		    RCLCPP_WARN(nh_.get_logger(), "Invalid or missing north_vector config — using default.");
    		    m_n << 1.0 / sqrt(3), 1.0 / sqrt(3), 1.0 / sqrt(3);
    		}

			// Convert gravity_vector
    		if (north_vec.size() == 3) {
    		    f_n = Eigen::Map<Eigen::Vector3d>(gravity_vec.data());
    		} else {
    		    RCLCPP_WARN(nh_.get_logger(), "Invalid or missing gravity_vector config — using default.");
    		    f_n << 0.0, 0.0, 9.81;
    		}
    		
			Eigen::Matrix6d P0 = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P_vec.data());
			Eigen::Matrix6d Q = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q_vec.data());
    		Eigen::Matrix6d R = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(R_vec.data());

    		attitude_ = new state_estimator::AttitudeBiasXKF(t0, xhat_estimated, P0, Q, R, f_n, m_n, ki, kp);

    		// Set up ROS interfaces
    		imu_sub_ = new rclcpp::Subscription<sensor_msgs::msg::imu>::SharedPtr(nh_.subscribe(imu_topic, 250, &AttitudeEstimationPlugin::callback_imu, this));
    		pub_ = new rclcpp::Publisher<state_estimator_msgs::msg::attitude>::SharedPtr(nh_.advertise<state_estimator_msgs::msg::attitude>(pub_topic, 1));

    		RCLCPP_INFO_STREAM(nh_.get_logger(), "AttitudeEstimationPlugin initialized with topic '" << imu_topic << "'");
  				
		}


		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback_imu 
		(
			const sensor_msgs::msg::Imu::ConstPtr& imu
		)
		{
			Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
        	Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);	
			computeAttitude(omega,acc);		
		}

		void computeAttitude(Eigen::Vector3d &omega,Eigen::Vector3d &acc)
		{
			/* to take the time from a rosbag:
			$ roscore
			$ rosparam set use_sim_time true
			$ rosbag play name_of_the_bag.bag --clock
			*/
	
			if (begin)
			{
				time_begin_ = this->node_.now().seconds();
				begin = false;
			}		

			time_ = this->node_.now().seconds() - time_begin_;

      		quat_est.w() = xhat_estimated(0);
      		quat_est.vec() << xhat_estimated(1),xhat_estimated(2),xhat_estimated(3);

      		f_b = b_R_imu*acc;
      		m_b = iit::commons::quatToRotMat(quat_est)*m_n;

      		z << f_b, m_b; 
	
			attitude_->update(time_,b_R_imu*omega, z);

      		xhat_estimated = attitude_->getX();
			
			xdot = attitude_->calc_f(time_,xhat_estimated,b_R_imu*omega);
			quat_dot.w() = xdot(0);
			quat_dot.vec() << xdot(1),xdot(2),xdot(3);
			omega_filt = iit::commons::quatToOmega(quat_est,quat_dot);

			// // publishing
			msg_.header.stamp = this->node_.now();

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

		state_estimator::AttitudeBiasXKF *attitude_;
		Eigen::Matrix<double,7,1> xhat_estimated;
		Eigen::Matrix<double,6,6> P0;
		Eigen::Matrix<double,6,6> Q;
		Eigen::Matrix<double,6,6> R;
		Eigen::Matrix<double,3,1> f_n; 
		Eigen::Matrix<double,3,1> m_n; 
    	Eigen::Matrix3d b_R_imu;
		double ki; 
		double kp; 
		double t0; 
	
		rclcpp::Subscription *imu_sub_;
		rclcpp::Publisher *pub_;

    	Eigen::Quaterniond quat_est;
    	Eigen::Vector3d f_b;
    	Eigen::Vector3d m_b; 
    	Eigen::Vector6d z;
    	Eigen::Vector7d xdot;
    	Eigen::Quaterniond quat_dot;
    	Eigen::Vector3d omega_filt;
    	Eigen::Vector3d euler_radians;
    	Eigen::Vector3d euler_degrees;

		state_estimator_msgs::msg::attitude msg_;
	
		double time_{};
		bool begin{true};
		double time_begin_;

	}; // end class AttitudeEstimationPlugin

} // end namespace state_estimator_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::AttitudeEstimationPlugin, state_estimator_plugins::PluginBase)
