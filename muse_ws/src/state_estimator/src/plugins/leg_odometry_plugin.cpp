#include <iit/commons/geometry/rotations.h>

#include "state_estimator/Models/joint_state_snapshot.hpp"
#include "state_estimator/Models/leg_odometry.hpp"
#include "state_estimator/plugin.hpp"
#include "state_estimator_msgs/ContactDetection.h"
#include "state_estimator_msgs/LegOdometry.h"
#include "state_estimator_msgs/attitude.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <anymal_msgs/AnymalState.h>

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class LegOdometryPlugin : public PluginBase
{
public:
	LegOdometryPlugin():
		imu_sub_(nullptr),
		anymal_state_sub_(nullptr),
		contact_sub_(nullptr),
		attitude_sub_(nullptr),
		pub_(nullptr)
	{ }

	~LegOdometryPlugin()
	{
		if (imu_sub_!=nullptr) delete(imu_sub_);
		if (anymal_state_sub_!=nullptr) delete(anymal_state_sub_);
		if (contact_sub_!=nullptr) delete(contact_sub_);
		if (attitude_sub_!=nullptr) delete(attitude_sub_);
		if (pub_!=nullptr) delete(pub_);
	}

	std::string getName() override { return std::string("LegOdometry"); }
	std::string getDescription() override { return std::string("Leg Odometry Plugin"); }

	void initialize_() override
	{
		std::string urdf_path_param;
		nh_.param("leg_odometry_plugin/urdf_path", urdf_path_param, std::string(""));
		if (urdf_path_param.empty())
		{
			ROS_ERROR("URDF path is not set in parameter server.");
			return;
		}

		const std::string resolved_path = resolveRosFind(urdf_path_param);
		ROS_INFO_STREAM("Loading URDF from: " << resolved_path);

		std::vector<double> base_R_imu_vec;
		nh_.param("leg_odometry_plugin/base_R_imu", base_R_imu_vec, defaultBaseRImuVector());
		if (base_R_imu_vec.size() == 9)
		{
			base_R_imu_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(base_R_imu_vec.data());
		}
		else
		{
			ROS_WARN("leg_odometry_plugin/base_R_imu is not the correct size (should be 9). Using offline ANYmal-D default.");
			const std::vector<double> default_base_R_imu = defaultBaseRImuVector();
			base_R_imu_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(default_base_R_imu.data());
		}

		std::vector<std::string> foot_frame_names;
		nh_.param("leg_odometry_plugin/foot_frame_names", foot_frame_names, defaultFootFrameNames());
		if (foot_frame_names.size() != 4)
		{
			ROS_WARN("leg_odometry_plugin/foot_frame_names must contain exactly 4 names. Using ANYmal defaults.");
			foot_frame_names = defaultFootFrameNames();
		}

		std::vector<std::string> joint_names;
		nh_.param("leg_odometry_plugin/joint_names", joint_names, defaultJointNames());
		if (joint_names.size() != 12)
		{
			ROS_WARN("leg_odometry_plugin/joint_names must contain exactly 12 names. Using ANYmal defaults.");
			joint_names = defaultJointNames();
		}

		std::string base_frame_name;
		nh_.param("leg_odometry_plugin/base_frame_name", base_frame_name, std::string("base"));

		model_configured_ = leg_odometry_.configure(resolved_path, foot_frame_names, joint_names, base_frame_name);
		if (!model_configured_)
		{
			ROS_ERROR_STREAM("Failed to configure leg odometry model: " << leg_odometry_.lastError());
			return;
		}
		ROS_INFO("Leg odometry Pinocchio model configured successfully.");

		std::string imu_topic, anymal_state_topic, contact_topic, attitude_topic, pub_topic;
		nh_.param("leg_odometry_plugin/imu_topic", imu_topic, std::string("/anymal/imu"));
		nh_.param("leg_odometry_plugin/anymal_state_topic", anymal_state_topic, std::string("/anymal/state"));
		nh_.param("leg_odometry_plugin/contact_topic", contact_topic, std::string("/state_estimator/contact_detection"));
		nh_.param("leg_odometry_plugin/attitude_topic", attitude_topic, std::string("/state_estimator/attitude"));
		nh_.param("leg_odometry_plugin/pub_topic", pub_topic, std::string("leg_odometry"));

		imu_sub_ = new ros::Subscriber(nh_.subscribe(imu_topic, 250, &LegOdometryPlugin::imuCallback, this));
		anymal_state_sub_ = new ros::Subscriber(nh_.subscribe(anymal_state_topic, 250, &LegOdometryPlugin::anymalStateCallback, this));
		contact_sub_ = new ros::Subscriber(nh_.subscribe(contact_topic, 250, &LegOdometryPlugin::contactCallback, this));
		attitude_sub_ = new ros::Subscriber(nh_.subscribe(attitude_topic, 250, &LegOdometryPlugin::attitudeCallback, this));
		pub_ = new ros::Publisher(nh_.advertise<state_estimator_msgs::LegOdometry>(pub_topic, 250));

		ROS_INFO_STREAM("LegOdometryPlugin reading ANYmal joint states from topic " << anymal_state_topic);
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

	void anymalStateCallback(const state_estimator_msgs::AnymalState::ConstPtr& anymal_state)
	{
		latest_joint_state_ = jointSnapshotFromAnymalState(*anymal_state);
		latest_joint_state_stamp_ = stampFromAnymalState(*anymal_state);
		has_joint_state_ = true;
	}

	void attitudeCallback(const state_estimator_msgs::attitude::ConstPtr& attitude)
	{
		latest_attitude_ = *attitude;
		has_attitude_ = true;
	}

	void contactCallback(const state_estimator_msgs::ContactDetection::ConstPtr& contact)
	{
		latest_contact_ = *contact;
		has_contact_ = true;
		publishLegOdometry();
	}

	void publishLegOdometry()
	{
		if (!model_configured_ || !has_imu_ || !has_joint_state_ || !has_contact_ || !has_attitude_)
		{
			ROS_WARN_THROTTLE(
				2.0,
				"LegOdometryPlugin waiting for inputs: imu=%d anymal_state=%d contact=%d attitude=%d",
				has_imu_,
				has_joint_state_,
				has_contact_,
				has_attitude_
			);
			return;
		}

		const Eigen::Vector3d omega_imu(
			latest_imu_.angular_velocity.x,
			latest_imu_.angular_velocity.y,
			latest_imu_.angular_velocity.z
		);
		const Eigen::Vector3d omega_base = base_R_imu_ * omega_imu;

		Eigen::Quaterniond quat_est;
		quat_est.w() = latest_attitude_.quaternion[0];
		quat_est.vec() << latest_attitude_.quaternion[1], latest_attitude_.quaternion[2], latest_attitude_.quaternion[3];
		if (quat_est.norm() > 1e-9)
		{
			quat_est.normalize();
		}
		else
		{
			quat_est.setIdentity();
		}

		const Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();

		const std::array<bool, 4> stance{{
			static_cast<bool>(latest_contact_.stance_lf),
			static_cast<bool>(latest_contact_.stance_rf),
			static_cast<bool>(latest_contact_.stance_lh),
			static_cast<bool>(latest_contact_.stance_rh)
		}};

		state_estimator::LegOdometryEstimate estimate;
		if (!leg_odometry_.update(latest_joint_state_, omega_base, stance, w_R_b, estimate))
		{
			ROS_WARN_THROTTLE(1.0, "LegOdometryPlugin update failed: %s", leg_odometry_.lastError().c_str());
			return;
		}

		msg_.header.stamp = outputStamp();
		for (int j = 0; j < 3; ++j)
		{
			msg_.lin_vel_lf[j] = estimate.base_velocity_from_foot_base[0][j];
			msg_.lin_vel_rf[j] = estimate.base_velocity_from_foot_base[1][j];
			msg_.lin_vel_lh[j] = estimate.base_velocity_from_foot_base[2][j];
			msg_.lin_vel_rh[j] = estimate.base_velocity_from_foot_base[3][j];
			msg_.base_velocity[j] = estimate.base_velocity_world[j];
		}
		msg_.stance_count = static_cast<uint8_t>(estimate.stance_count);
		msg_.base_height_valid = estimate.base_height_valid;
		msg_.base_height = estimate.base_height;
		for (std::size_t leg = 0; leg < stance.size(); ++leg)
		{
			msg_.stance[leg] = stance[leg];
			msg_.base_to_foot_position_world_z[leg] =
				(w_R_b * estimate.kinematics.foot_position_base[leg]).z();
		}

		pub_->publish(msg_);
	}

	ros::Time outputStamp() const
	{
		if (!latest_contact_.header.stamp.isZero()) return latest_contact_.header.stamp;
		if (!latest_joint_state_stamp_.isZero()) return latest_joint_state_stamp_;
		if (!latest_imu_.header.stamp.isZero()) return latest_imu_.header.stamp;
		return ros::Time::now();
	}

	static ros::Time stampFromAnymalState(const state_estimator_msgs::AnymalState& anymal_state)
	{
		if (!anymal_state.header.stamp.isZero()) return anymal_state.header.stamp;
		if (!anymal_state.joints.header.stamp.isZero()) return anymal_state.joints.header.stamp;
		if (!anymal_state.pose.header.stamp.isZero()) return anymal_state.pose.header.stamp;
		if (!anymal_state.twist.header.stamp.isZero()) return anymal_state.twist.header.stamp;
		return ros::Time();
	}

	static state_estimator::JointStateSnapshot jointSnapshotFromAnymalState(
		const state_estimator_msgs::AnymalState& anymal_state)
	{
		state_estimator::JointStateSnapshot joints;
		joints.name = anymal_state.joints.name;
		joints.position = anymal_state.joints.position;
		joints.velocity = anymal_state.joints.velocity;
		joints.acceleration = anymal_state.joints.acceleration;
		joints.effort = anymal_state.joints.effort;
		return joints;
	}

	static std::string resolveRosFind(const std::string& input)
	{
		std::string resolved = input;
		const std::string find_token = "$(find ";
		const std::size_t token_pos = resolved.find(find_token);
		if (token_pos == std::string::npos)
		{
			return resolved;
		}

		const std::size_t package_start = token_pos + find_token.length();
		const std::size_t package_end = resolved.find(")", package_start);
		if (package_end == std::string::npos)
		{
			return resolved;
		}

		const std::string package_name = resolved.substr(package_start, package_end - package_start);
		const std::string package_path = ros::package::getPath(package_name);
		if (package_path.empty())
		{
			return resolved;
		}

		resolved.replace(token_pos, package_end - token_pos + 1, package_path);
		return resolved;
	}

	static std::vector<std::string> defaultFootFrameNames()
	{
		return {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
	}

	static std::vector<std::string> defaultJointNames()
	{
		return {
			"LF_HAA", "LF_HFE", "LF_KFE",
			"RF_HAA", "RF_HFE", "RF_KFE",
			"LH_HAA", "LH_HFE", "LH_KFE",
			"RH_HAA", "RH_HFE", "RH_KFE"
		};
	}

	static std::vector<double> defaultBaseRImuVector()
	{
		return {
			-1.0, 0.0, 0.0,
			 0.0, 1.0, 0.0,
			 0.0, 0.0, -1.0
		};
	}

	ros::Subscriber *imu_sub_;
	ros::Subscriber *anymal_state_sub_;
	ros::Subscriber *contact_sub_;
	ros::Subscriber *attitude_sub_;
	ros::Publisher *pub_;

	state_estimator_msgs::LegOdometry msg_;
	sensor_msgs::Imu latest_imu_;
	state_estimator::JointStateSnapshot latest_joint_state_;
	ros::Time latest_joint_state_stamp_;
	state_estimator_msgs::ContactDetection latest_contact_;
	state_estimator_msgs::attitude latest_attitude_;

	state_estimator::LegOdometryModel leg_odometry_;

	bool model_configured_{false};
	bool has_imu_{false};
	bool has_joint_state_{false};
	bool has_contact_{false};
	bool has_attitude_{false};
	Eigen::Matrix3d base_R_imu_{Eigen::Matrix3d::Identity()};
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::LegOdometryPlugin, state_estimator_plugins::PluginBase)
