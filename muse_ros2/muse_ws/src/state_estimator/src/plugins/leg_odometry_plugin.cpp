#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "state_estimator_msgs/msg/leg_odometry.hpp"
#include "state_estimator_msgs/msg/joint_state_with_acceleration.hpp" 
#include "state_estimator_msgs/msg/contact_detection.hpp"
#include "state_estimator_msgs/msg/attitude.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

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
	state_estimator_msgs::msg::JointStateWithAcceleration,
    state_estimator_msgs::msg::ContactDetection,
    state_estimator_msgs::msg::Attitude
> 
ApproximateTimePolicy;

typedef message_filters::sync_policies::ExactTime
<
	sensor_msgs::msg::Imu,
	state_estimator_msgs::msg::JointStateWithAcceleration,
    state_estimator_msgs::msg::ContactDetection,
    state_estimator_msgs::msg::Attitude
> 
ExactTimePolicy;

#define MySyncPolicy ApproximateTimePolicy

//  This class calculates the base velocity as estimated from the leg kinematics. These are NOT the velocity of the legs.
	class LegOdometryPlugin : public PluginBase
	{
	public:
		LegOdometryPlugin(): 
			imu_sub_(nullptr), 
			joint_state_sub_(nullptr), 
			contact_sub_(nullptr),
			attitude_sub_(nullptr),
			pub_(nullptr), 
			sync_(nullptr) 
		{ } 
	
		~LegOdometryPlugin() 
		{
			// Smart pointers will be automatically destroyed
		}		std::string getName() override { return std::string("LegOdometry"); }
		std::string getDescription() override { return std::string("Leg Odometry Plugin"); }

		void initialize_() override {

			RCLCPP_INFO(node_->get_logger(), "Initializing LegOdometryPlugin...");

			std::string urdf_path_param;
			// node_->declare_parameter("leg_odometry_plugin.urdf_path", "");
			// urdf_path_param = node_->get_parameter("leg_odometry_plugin.urdf_path").as_string();
			urdf_path_param = "/root/muse_ws/muse_ros2_ws/src/state_estimator/urdfs/anymal.urdf";
			RCLCPP_INFO_STREAM(node_->get_logger(), "URDF path parameter: " << urdf_path_param);

			if (urdf_path_param.empty()) {
				RCLCPP_ERROR(node_->get_logger(), "URDF path is not set in parameter server.");
				return;
			}

			// Resolve $(find ...) manually if present
			std::string resolved_path = urdf_path_param;
			std::string find_token = "$(find ";
			if (resolved_path.find(find_token) != std::string::npos) {
				size_t start = resolved_path.find(find_token) + find_token.length();
				size_t end = resolved_path.find(")", start);
				std::string package_name = resolved_path.substr(start, end - start);
				std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
				resolved_path.replace(resolved_path.find(find_token), end - resolved_path.find(find_token) + 1, package_path);
			}

			RCLCPP_INFO_STREAM(node_->get_logger(), "Loading URDF from: " << resolved_path);

			try {
				pinocchio::urdf::buildModel(resolved_path, model_);
				data_ = pinocchio::Data(model_);
				RCLCPP_INFO(node_->get_logger(), "URDF loaded into Pinocchio model successfully.");
			} catch (const std::exception& e) {
				RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to load URDF: " << e.what());
			}

			std::vector<double> base_R_imu_vec;
			node_->declare_parameter("leg_odometry_plugin.base_R_imu", std::vector<double>(9, 0.0));
			try {
				base_R_imu_vec = node_->get_parameter("leg_odometry_plugin.base_R_imu").get_value<std::vector<double>>();
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

			std::string imu_topic, joint_states_topic, contact_topic, attitude_topic, pub_topic ;
			
			// Get topic names from parameters
			RCLCPP_INFO(node_->get_logger(), "Declaring topic parameters...");
			
			node_->declare_parameter("leg_odometry_plugin.imu_topic", "/sensors/imu");
			node_->declare_parameter("leg_odometry_plugin.joint_states_topic", "/state_estimator/joint_states");
			node_->declare_parameter("leg_odometry_plugin.contact_topic", "/state_estimator/contact_detection");
			node_->declare_parameter("leg_odometry_plugin.attitude_topic", "/attitude");
			node_->declare_parameter("leg_odometry_plugin.pub_topic", "/state_estimator/leg_odometry");
			
			try {
				imu_topic = node_->get_parameter("leg_odometry_plugin.imu_topic").as_string();
				RCLCPP_INFO_STREAM(node_->get_logger(), "imu_topic: " << imu_topic);
				
				joint_states_topic = node_->get_parameter("leg_odometry_plugin.joint_states_topic").as_string();
				RCLCPP_INFO_STREAM(node_->get_logger(), "joint_states_topic: " << joint_states_topic);
				
				contact_topic = node_->get_parameter("leg_odometry_plugin.contact_topic").as_string();
				RCLCPP_INFO_STREAM(node_->get_logger(), "contact_topic: " << contact_topic);
				
				attitude_topic = node_->get_parameter("leg_odometry_plugin.attitude_topic").as_string();
				RCLCPP_INFO_STREAM(node_->get_logger(), "attitude_topic: " << attitude_topic);
				
				pub_topic = node_->get_parameter("leg_odometry_plugin.pub_topic").as_string();
				RCLCPP_INFO_STREAM(node_->get_logger(), "pub_topic: " << pub_topic);
			} catch (const std::exception& e) {
				RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get topic parameters: " << e.what());
				return;
			}
			
			// Set up subscribers using smart pointers
			imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(node_, imu_topic);
			joint_state_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::JointStateWithAcceleration>>(node_, joint_states_topic);
			contact_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::ContactDetection>>(node_, contact_topic);
			attitude_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>>(node_, attitude_topic);

			sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(100), *imu_sub_, *joint_state_sub_, *contact_sub_, *attitude_sub_); 
			sync_->registerCallback(std::bind(&LegOdometryPlugin::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

			pub_ = node_->create_publisher<state_estimator_msgs::msg::LegOdometry>(pub_topic, 250);

		}
		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback
		(
			const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
			const state_estimator_msgs::msg::JointStateWithAcceleration::ConstSharedPtr& js,
			const state_estimator_msgs::msg::ContactDetection::ConstSharedPtr& contact,
			const state_estimator_msgs::msg::Attitude::ConstSharedPtr& attitude
		)
		{
			// Robot joint states
			// Fill q and v from joint state
			Eigen::VectorXd q(model_.nq);
			Eigen::VectorXd v(model_.nv);

			if (js->position.size() != model_.nq || js->velocity.size() != model_.nv) {
				RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Mismatch in joint state size");
				return;
			}

			for (size_t i = 0; i < model_.nq; ++i)
				q[i] = js->position[i];

			for (size_t i = 0; i < model_.nv; ++i)
				v[i] = js->velocity[i];
			
			omega << imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z;
			stance_lf = contact->stance_lf;
			stance_rf = contact->stance_rf;
			stance_lh = contact->stance_lh;
			stance_rh = contact->stance_rh;

			std::vector<Eigen::Vector3d> foot_velocities;

			// Compute the forward kinematics first
			pinocchio::forwardKinematics(model_, data_, q, v);
			pinocchio::updateFramePlacements(model_, data_);
			std::vector<Eigen::Vector3d> foot_vels;

			for (size_t i = 0; i < feet_frame_names.size(); ++i) {
				const auto& foot_name = feet_frame_names[i];
				std::size_t frame_id = model_.getFrameId(foot_name);
			
				// Get spatial velocity in LOCAL_WORLD_ALIGNED frame
				pinocchio::Motion foot_vel_global = pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

				// Get position of the foot in the base frame
				Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();  // Position of foot in world
				Eigen::Vector3d omega_rotated = base_R_imu_ * omega;

				// Compute velocity contribution from base angular motion: ω x r
				Eigen::Vector3d omega_cross_r = omega_rotated.cross(foot_pos_base);

				// Compute linear velocity of the foot relative to base
				Eigen::Vector3d rel_vel = -(foot_vel_global.linear() - omega_cross_r);
				foot_vels.push_back(rel_vel);
			
			}

			Eigen::Vector3d lin_leg_lf = foot_vels[0];
			Eigen::Vector3d lin_leg_rf = foot_vels[1];  
			Eigen::Vector3d lin_leg_lh = foot_vels[2];
			Eigen::Vector3d lin_leg_rh = foot_vels[3];

			double sum_stance = stance_lf + stance_rf + stance_lh + stance_rh;
			Eigen::Vector3d base_velocity = (stance_lf*lin_leg_lf + stance_rf*lin_leg_rf + stance_lh*lin_leg_lh + stance_rh*lin_leg_rh)/(sum_stance + 1e-5);

			Eigen::Quaterniond quat_est;
			quat_est.w() = attitude->quaternion[0];
			quat_est.vec() << attitude->quaternion[1], attitude->quaternion[2], attitude->quaternion[3];
			Eigen::Matrix3d w_R_b = quat_est.toRotationMatrix().transpose();

			base_velocity = w_R_b*base_velocity;

			// publishing	
			msg_.header.stamp = node_->get_clock()->now();

			for (int j=0;j<3;j++) {
				msg_.lin_vel_lf[j] = lin_leg_lf.data()[j];
				msg_.lin_vel_rf[j] = lin_leg_rf.data()[j];
				msg_.lin_vel_lh[j] = lin_leg_lh.data()[j];
				msg_.lin_vel_rh[j] = lin_leg_rh.data()[j];
				msg_.base_velocity[j] = base_velocity.data()[j];
			}

			pub_->publish(msg_);

		} // end callback
	private:
	
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;
		std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::JointStateWithAcceleration>> joint_state_sub_;
		std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::ContactDetection>> contact_sub_;
		std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>> attitude_sub_;
		std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

		rclcpp::Publisher<state_estimator_msgs::msg::LegOdometry>::SharedPtr pub_;

		state_estimator_msgs::msg::LegOdometry msg_;        pinocchio::Model model_;
        pinocchio::Data data_;
        std::vector<std::string> feet_frame_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};   // Update with your actual link names
        // std::vector<std::string> feet_frame_names = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};   // Aliengo robot

        bool model_loaded_{false};

        Eigen::Vector3d omega;
        Eigen::Vector3d base_omega;
		bool stance_lf;
		bool stance_rf;
		bool stance_lh;
		bool stance_rh;
        Eigen::Matrix3d base_R_imu_;

	}; // end class LegOdometryPlugin

} //end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::LegOdometryPlugin, state_estimator_plugins::PluginBase)
