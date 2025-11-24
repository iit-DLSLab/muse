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
#include "state_estimator_msgs/msg/base_height.hpp"  // initial base height message
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
		SensorFusionPlugin() = default;

		~SensorFusionPlugin() override {
		    if (sensor_fusion_ != nullptr) {
		        delete sensor_fusion_;
		    }
		}

		std::string getName() override { return std::string("SensorFusion"); }
		std::string getDescription() override { return std::string("Sensor Fusion Plugin"); }

		void initialize_() override
		{
			t0 = 0.0;
			xhat_estimated << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			omega.setZero(); 
			v_b.setZero();   
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

			// Gravity vector in world frame (default Z-up world, gravity down is +9.81 along +Z specific force sense)
			// We will use u = R * f_b + g_w, so at rest u ≈ 0 if g_w matches world gravity sign convention
			std::vector<double> gravity_vec = nh->declare_parameter<std::vector<double>>("sensor_fusion_plugin.gravity_vector", {0.0, 0.0, 9.81});
			if (gravity_vec.size() == 3) {
				gravity_w_ << gravity_vec[0], gravity_vec[1], gravity_vec[2];
				RCLCPP_INFO(nh->get_logger(), "sensor_fusion_plugin.gravity_vector: [%.3f, %.3f, %.3f]",
					gravity_w_.x(), gravity_w_.y(), gravity_w_.z());
			} else {
				RCLCPP_WARN(nh->get_logger(), "gravity_vector size != 3; using default [0,0,9.81]");
				gravity_w_ << 0.0, 0.0, 9.81;
			}

			const std::vector<double> base_R_imu_vec = nh->declare_parameter("sensor_fusion_plugin.base_R_imu", std::vector<double>(9, 0.0));
			if (base_R_imu_vec.size() == 9)
			{
				base_R_imu_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(base_R_imu_vec.data());
				std::stringstream ss;
				ss << base_R_imu_;
				RCLCPP_INFO(nh->get_logger(), "Loaded base_R_imu:\n%s", ss.str().c_str());
			}
			else
			{
				RCLCPP_WARN(nh->get_logger(), "base_R_imu is not the correct size (should be 9). Using identity.");
				base_R_imu_ = Eigen::Matrix3d::Identity();
			}

			//Declare parameters
			const std::string imu_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.imu_topic", "/sensors/imu");
			const std::string attitude_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.attitude_topic", "/state_estimator/attitude");
			const std::string leg_odom_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.leg_odometry_topic", "/sensors/leg_odometry");
			const std::string pub_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.pub_topic", "/sensors/odometry");
			const std::string base_height_topic = nh->declare_parameter<std::string>("sensor_fusion_plugin.base_height_topic", "/state_estimator/base_height");

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

			// One-shot BaseHeight subscription (first message seeds z position)
			base_height_sub_ = nh->create_subscription<state_estimator_msgs::msg::BaseHeight>(
				base_height_topic,
				sensor_qos,
				[this](const state_estimator_msgs::msg::BaseHeight::SharedPtr msg) {
					if (!initial_height_set_) {
						xhat_estimated(2) = msg->height; // set z component
						if (sensor_fusion_) {
							sensor_fusion_->setState(xhat_estimated);
						}
						initial_height_set_ = true;
						RCLCPP_INFO(this->node_->get_logger(), "Initial height received: %.3f m (frame=%s)", msg->height, msg->header.frame_id.c_str());
					}
				});

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
			Eigen::Matrix3d b_R_w = iit::commons::quatToRotMat(quat_est);
			// Use body->world rotation
			Eigen::Matrix3d w_R_b = b_R_w.transpose();

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
				if (!initial_height_set_) {
					RCLCPP_DEBUG(this->node_->get_logger(), "Initial height not yet received; using z=%.3f until BaseHeight arrives", xhat_estimated(2));
				}
			}

			time_ = this->node_->get_clock()->now().seconds() - time_begin_;

			// reading acceleration from imu
			Eigen::Vector3d f_b = base_R_imu_ * acc;

			// World specific force (should be ≈ gravity when stationary)
			Eigen::Vector3d w_f = w_R_b * f_b;

			// True world linear acceleration (control input)
			Eigen::Vector3d u = w_f - gravity_w_;

			// DEBUG: Log key values every 50000 iterations
			static int debug_counter = 0;
			if (debug_counter++ % 5000 == 0) {
				RCLCPP_INFO(this->node_->get_logger(), 
					"=== DEBUG [t=%.3f] ===\n"
					"Raw IMU acc: [%.3f, %.3f, %.3f]\n"
					"f_b (base frame): [%.3f, %.3f, %.3f]\n"
					"w_R_b*f_b (world specific force): [%.3f, %.3f, %.3f]\n"
					"gravity_w: [%.3f, %.3f, %.3f]\n"
					"u = w_R_b*f_b - gravity_w: [%.3f, %.3f, %.3f]\n"
					"v_b (leg odom): [%.3f, %.3f, %.3f]\n"
					"w_v_b (world vel): [%.3f, %.3f, %.3f]\n"
					"State pos: [%.3f, %.3f, %.3f]\n"
					"State vel: [%.3f, %.3f, %.3f]",
					time_,
					acc.x(), acc.y(), acc.z(),
					f_b.x(), f_b.y(), f_b.z(),
					w_f.x(), w_f.y(), w_f.z(),
					gravity_w_.x(), gravity_w_.y(), gravity_w_.z(),
					u.x(), u.y(), u.z(),
					v_b.x(), v_b.y(), v_b.z(),
					(w_R_b * v_b).x(), (w_R_b * v_b).y(), (w_R_b * v_b).z(),
					xhat_estimated(0), xhat_estimated(1), xhat_estimated(2),
					xhat_estimated(3), xhat_estimated(4), xhat_estimated(5));
			}

			// prediction
			sensor_fusion_->predict(time_, u);
            Eigen::Vector3d z_proprio;
			//Convert leg odometry from body to world frame
            z_proprio << w_R_b * v_b;

            // DIAGNOSTIC: Check Kalman gain behavior
            Eigen::Vector3d innovation = z_proprio - xhat_estimated.tail<3>();
            static int tune_counter = 0;
            if (tune_counter++ % 50000 == 0) {
                RCLCPP_INFO(this->node_->get_logger(),
                    "=== TUNING DEBUG ===\n"
                    "Measurement (leg odom vel): [%.3f, %.3f, %.3f]\n"
                    "Predicted vel: [%.3f, %.3f, %.3f]\n"
                    "Innovation (meas-pred): [%.3f, %.3f, %.3f] norm=%.3f",
                    z_proprio.x(), z_proprio.y(), z_proprio.z(),
                    xhat_estimated(3), xhat_estimated(4), xhat_estimated(5),
                    innovation.x(), innovation.y(), innovation.z(), innovation.norm());
            }

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
		rclcpp::Subscription<state_estimator_msgs::msg::BaseHeight>::SharedPtr base_height_sub_;

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

		nav_msgs::msg::Odometry msg_;

		double time_{};
		bool begin{true};
		bool initial_height_set_{false};
		double time_begin_;
		Eigen::Vector3d v_b;
		Eigen::Quaterniond quat_est;
		Eigen::Vector3d omega;
		double yaw;
		Eigen::Vector3d rpy;
		Eigen::Matrix3d w_R_b;
		Eigen::Matrix3d base_R_imu_;
		// World gravity vector (loaded from parameter or defaulted to [0,0,9.81])
		Eigen::Vector3d gravity_w_;

		Eigen::Matrix3d w_R_wb; // world frame

	}; // end class SensorFusionPlugin

} // end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::SensorFusionPlugin, state_estimator_plugins::PluginBase)
