#include "state_estimator/Models/sensor_fusion.hpp"
#include "state_estimator/plugin.hpp"

#include "state_estimator_msgs/LegOdometry.h"
#include "state_estimator_msgs/attitude.h"

#include <iit/commons/geometry/rotations.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace state_estimator_plugins
{

class SensorFusionPlugin : public PluginBase
{
public:
	SensorFusionPlugin():
		sensor_fusion_(nullptr),
		imu_sub_(nullptr),
		attitude_sub_(nullptr),
		leg_odom_sub_(nullptr),
		pub_(nullptr)
	{ }

	~SensorFusionPlugin()
	{
		if (sensor_fusion_!=nullptr) delete(sensor_fusion_);
		if (imu_sub_!=nullptr) delete(imu_sub_);
		if (attitude_sub_!=nullptr) delete(attitude_sub_);
		if (leg_odom_sub_!=nullptr) delete(leg_odom_sub_);
		if (pub_!=nullptr) delete(pub_);
	}

	std::string getName() override { return std::string("SensorFusion"); }
	std::string getDescription() override { return std::string("Sensor Fusion Plugin"); }

	void initialize_() override
	{
		t0 = 0.0;
		xhat_estimated << 0.0, 0.0, 0.45, 0.0, 0.0, 0.0;

		std::vector<double> P_vec, Q_vec, R_vec;
		nh_.param("sensor_fusion_plugin/P", P_vec, std::vector<double>(36, 0.0));
		nh_.param("sensor_fusion_plugin/Q", Q_vec, std::vector<double>(36, 0.0));
		nh_.param("sensor_fusion_plugin/R", R_vec, std::vector<double>(9, 0.0));

		if (P_vec.size() != 36 || Q_vec.size() != 36 || R_vec.size() != 9)
		{
			ROS_ERROR("P0, Q0, or R0 parameters have incorrect size. P and Q need 36 elements, R needs 9.");
			return;
		}

		P = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P_vec.data());
		Q = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q_vec.data());
		R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_vec.data());

		sensor_fusion_ = new state_estimator::KFSensorFusion(t0, xhat_estimated, P, Q, R, false, false);

		std::vector<double> base_R_imu_vec;
		nh_.param("attitude_estimation_plugin/base_R_imu", base_R_imu_vec, std::vector<double>(9, 0.0));
		if (base_R_imu_vec.size() == 9)
		{
			base_R_imu_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(base_R_imu_vec.data());
		}
		else
		{
			ROS_WARN("base_R_imu is not the correct size (should be 9). Using identity.");
			base_R_imu_ = Eigen::Matrix3d::Identity();
		}

		std::string imu_topic, attitude_topic, leg_odom_topic, pub_topic;
		nh_.param<std::string>("sensor_fusion_plugin/imu_topic", imu_topic, "/anymal/imu");
		nh_.param<std::string>("sensor_fusion_plugin/attitude_topic", attitude_topic, "/state_estimator/attitude");
		nh_.param<std::string>("sensor_fusion_plugin/leg_odometry_topic", leg_odom_topic, "/state_estimator/leg_odometry");
		nh_.param<std::string>("sensor_fusion_plugin/pub_topic", pub_topic, "sensor_fusion");
		nh_.param<std::string>("sensor_fusion_plugin/frame_id", frame_id_, "world");
		nh_.param<std::string>("sensor_fusion_plugin/child_frame_id", child_frame_id_, "base");
		nh_.param<bool>("sensor_fusion_plugin/z_height_correction/enabled", z_height_correction_enabled_, true);
		nh_.param<bool>("sensor_fusion_plugin/z_height_correction/assume_flat_ground", z_height_assume_flat_ground_, false);
		nh_.param<double>("sensor_fusion_plugin/z_height_correction/ground_height", z_height_ground_height_, 0.0);
		nh_.param<double>("sensor_fusion_plugin/z_height_correction/measurement_variance", z_height_measurement_variance_, 1.0e-8);
		nh_.param<double>("sensor_fusion_plugin/z_height_correction/max_innovation", z_height_max_innovation_, 0.25);
		nh_.param<double>("sensor_fusion_plugin/z_height_correction/filter_alpha", z_height_filter_alpha_, 1.0);

		int min_stance_legs = 1;
		nh_.param<int>("sensor_fusion_plugin/z_height_correction/min_stance_legs", min_stance_legs, 1);
		z_height_min_stance_legs_ = std::max(1, std::min(4, min_stance_legs));

		imu_sub_ = new ros::Subscriber(nh_.subscribe(imu_topic, 250, &SensorFusionPlugin::imuCallback, this));
		attitude_sub_ = new ros::Subscriber(nh_.subscribe(attitude_topic, 250, &SensorFusionPlugin::attitudeCallback, this));
		leg_odom_sub_ = new ros::Subscriber(nh_.subscribe(leg_odom_topic, 250, &SensorFusionPlugin::legOdometryCallback, this));
		pub_ = new ros::Publisher(nh_.advertise<nav_msgs::Odometry>(pub_topic, 250));

		ROS_INFO_STREAM(
			"SensorFusionPlugin initialized with imu=" << imu_topic
			<< ", attitude=" << attitude_topic
			<< ", leg_odometry=" << leg_odom_topic
		);
	}

	void shutdown_() override { }
	void pause_() override { }
	void resume_() override { }
	void reset_() override { }

private:
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
	{
		latest_imu_ = *imu;
		has_imu_ = true;
	}

	void attitudeCallback(const state_estimator_msgs::attitude::ConstPtr& attitude)
	{
		latest_attitude_ = *attitude;
		has_attitude_ = true;
	}

	void legOdometryCallback(const state_estimator_msgs::LegOdometry::ConstPtr& leg_odom)
	{
		latest_leg_odom_ = *leg_odom;
		has_leg_odom_ = true;
		callback_proprioception();
	}

	void callback_proprioception()
	{
		if (!has_imu_ || !has_attitude_ || !has_leg_odom_)
		{
			ROS_WARN_THROTTLE(
				2.0,
				"SensorFusionPlugin waiting for inputs: imu=%d attitude=%d leg_odometry=%d",
				has_imu_,
				has_attitude_,
				has_leg_odom_
			);
			return;
		}

		ROS_DEBUG_STREAM_THROTTLE(
			1.0,
			"SensorFusionPlugin received inputs: imu timestamp = " << latest_imu_.header.stamp
			<< ", attitude timestamp = " << latest_attitude_.header.stamp
			<< ", leg odometry timestamp = " << latest_leg_odom_.header.stamp
		);

		Eigen::Vector3d acc(
			latest_imu_.linear_acceleration.x,
			latest_imu_.linear_acceleration.y,
			latest_imu_.linear_acceleration.z
		);

		omega << latest_attitude_.angular_velocity[0], latest_attitude_.angular_velocity[1], latest_attitude_.angular_velocity[2];
		quat_est.w() = latest_attitude_.quaternion[0];
		quat_est.vec() << latest_attitude_.quaternion[1], latest_attitude_.quaternion[2], latest_attitude_.quaternion[3];
		rpy = iit::commons::quatToRPY(quat_est);
		w_R_b = iit::commons::quatToRotMat(quat_est).transpose();

		v_b << latest_leg_odom_.base_velocity[0], latest_leg_odom_.base_velocity[1], latest_leg_odom_.base_velocity[2];
		computeLinPosVel(acc, w_R_b, v_b, outputStamp());
	}

	void computeLinPosVel(Eigen::Vector3d& acc, Eigen::Matrix3d& w_R_b, Eigen::Vector3d& v_b, const ros::Time& stamp)
	{
		const ros::Time effective_stamp = stamp.isZero() ? ros::Time::now() : stamp;
		if (begin)
		{
			time_begin_ = effective_stamp.toSec();
			last_update_time_ = 0.0;
			w_R_wb = w_R_b;
			begin = false;
		}

		time_ = effective_stamp.toSec() - time_begin_;
		if (time_ < last_update_time_)
		{
			ROS_WARN_THROTTLE(1.0, "SensorFusionPlugin received out-of-order timestamps. Skipping update.");
			return;
		}

		Eigen::Vector3d f_b = base_R_imu_ * acc;
		Eigen::Vector3d gravity;
		gravity << 0.0, 0.0, -9.81;
		Eigen::Vector3d u = w_R_b * f_b + gravity;

		sensor_fusion_->predict(time_, u);

		Eigen::Vector3d z_proprio;
		z_proprio << v_b;
		sensor_fusion_->update(time_, z_proprio);
		applyZHeightCorrection();
		xhat_estimated = sensor_fusion_->getX();
		last_update_time_ = time_;

		msg_.header.stamp = effective_stamp;
		msg_.header.frame_id = frame_id_;
		msg_.child_frame_id = child_frame_id_;

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
	}

	void applyZHeightCorrection()
	{
		if (!z_height_correction_enabled_)
		{
			return;
		}

		if (z_height_assume_flat_ground_)
		{
			applyFlatGroundZHeightCorrection();
			return;
		}

		applyContactAnchorZCorrection();
	}

	void applyFlatGroundZHeightCorrection()
	{
		if (!latest_leg_odom_.base_height_valid)
		{
			return;
		}
		if (latest_leg_odom_.stance_count < z_height_min_stance_legs_)
		{
			return;
		}

		const double raw_measurement = latest_leg_odom_.base_height + z_height_ground_height_;
		if (!std::isfinite(raw_measurement))
		{
			return;
		}

		applyBoundedZMeasurement(raw_measurement);
	}

	void applyContactAnchorZCorrection()
	{
		const Eigen::Matrix<double, 6, 1> xhat = sensor_fusion_->getX();
		const double current_base_z = xhat(2);

		double z_measurement_sum = 0.0;
		std::size_t valid_measurements = 0;

		for (std::size_t leg = 0; leg < foot_anchor_world_z_.size(); ++leg)
		{
			const bool stance = static_cast<bool>(latest_leg_odom_.stance[leg]);
			const double base_to_foot_z = latest_leg_odom_.base_to_foot_position_world_z[leg];

			if (!stance)
			{
				foot_anchor_valid_[leg] = false;
				previous_foot_stance_[leg] = false;
				continue;
			}
			if (!std::isfinite(base_to_foot_z))
			{
				continue;
			}

			if (!previous_foot_stance_[leg] || !foot_anchor_valid_[leg])
			{
				foot_anchor_world_z_[leg] = current_base_z + base_to_foot_z;
				foot_anchor_valid_[leg] = true;
			}

			z_measurement_sum += foot_anchor_world_z_[leg] - base_to_foot_z;
			valid_measurements++;
			previous_foot_stance_[leg] = true;
		}

		if (valid_measurements < static_cast<std::size_t>(z_height_min_stance_legs_))
		{
			return;
		}

		applyBoundedZMeasurement(z_measurement_sum / static_cast<double>(valid_measurements));
	}

	void applyBoundedZMeasurement(double raw_measurement)
	{
		if (z_height_measurement_variance_ <= 0.0)
		{
			ROS_WARN_THROTTLE(2.0, "SensorFusionPlugin z height correction disabled by non-positive measurement variance.");
			return;
		}
		if (!std::isfinite(raw_measurement))
		{
			return;
		}

		const double alpha = std::max(0.0, std::min(1.0, z_height_filter_alpha_));
		if (!has_filtered_z_height_)
		{
			filtered_z_height_ = raw_measurement;
			has_filtered_z_height_ = true;
		}
		else
		{
			filtered_z_height_ = alpha * raw_measurement + (1.0 - alpha) * filtered_z_height_;
		}

		const double innovation = filtered_z_height_ - sensor_fusion_->getX()(2);
		if (z_height_max_innovation_ > 0.0 && std::abs(innovation) > z_height_max_innovation_)
		{
			filtered_z_height_ = sensor_fusion_->getX()(2) + std::copysign(z_height_max_innovation_, innovation);
			ROS_WARN_THROTTLE(
				1.0,
				"SensorFusionPlugin clamped z height correction: innovation %.3f exceeds gate %.3f",
				innovation,
				z_height_max_innovation_
			);
		}

		sensor_fusion_->updateZPosition(time_, filtered_z_height_, z_height_measurement_variance_);
	}

	ros::Time outputStamp() const
	{
		if (!latest_leg_odom_.header.stamp.isZero()) return latest_leg_odom_.header.stamp;
		if (!latest_attitude_.header.stamp.isZero()) return latest_attitude_.header.stamp;
		if (!latest_imu_.header.stamp.isZero()) return latest_imu_.header.stamp;
		return ros::Time::now();
	}

	state_estimator::KFSensorFusion *sensor_fusion_;

	double t0;
	Eigen::Matrix<double, 6, 1> xhat_estimated;
	Eigen::Matrix<double, 6, 6> P;
	Eigen::Matrix<double, 6, 6> Q;
	Eigen::Matrix<double, 3, 3> R;

	ros::Subscriber *imu_sub_;
	ros::Subscriber *attitude_sub_;
	ros::Subscriber *leg_odom_sub_;
	ros::Publisher *pub_;

	sensor_msgs::Imu latest_imu_;
	state_estimator_msgs::attitude latest_attitude_;
	state_estimator_msgs::LegOdometry latest_leg_odom_;
	nav_msgs::Odometry msg_;
	std::string frame_id_;
	std::string child_frame_id_;

	bool has_imu_{false};
	bool has_attitude_{false};
	bool has_leg_odom_{false};
	double time_{};
	double last_update_time_{0.0};
	bool begin{true};
	double time_begin_{0.0};
	Eigen::Vector3d v_b;
	Eigen::Quaterniond quat_est;
	Eigen::Vector3d omega;
	Eigen::Vector3d rpy;
	Eigen::Matrix3d w_R_b;
	Eigen::Matrix3d base_R_imu_;

	Eigen::Matrix3d w_R_wb;
	bool z_height_correction_enabled_{true};
	bool z_height_assume_flat_ground_{false};
	int z_height_min_stance_legs_{1};
	double z_height_ground_height_{0.0};
	double z_height_measurement_variance_{1.0e-8};
	double z_height_max_innovation_{0.25};
	double z_height_filter_alpha_{1.0};
	bool has_filtered_z_height_{false};
	double filtered_z_height_{0.0};
	std::array<bool, 4> foot_anchor_valid_{{false, false, false, false}};
	std::array<bool, 4> previous_foot_stance_{{false, false, false, false}};
	std::array<double, 4> foot_anchor_world_z_{{0.0, 0.0, 0.0, 0.0}};
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::SensorFusionPlugin, state_estimator_plugins::PluginBase)
