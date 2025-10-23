// Pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "iit/commons/geometry/rotations.h"


// Core
#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "state_estimator_msgs/msg/leg_odometry.hpp"
#include "state_estimator_msgs/msg/joint_state_with_acceleration.hpp" 
#include "state_estimator_msgs/msg/contact_detection.hpp"
#include "state_estimator_msgs/msg/attitude.hpp"

// ROS 2 messages
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

// ROS 2 message_filters
#include <message_filters/time_synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

#include <unordered_map>


namespace state_estimator_plugins
{

using ImuMsg = sensor_msgs::msg::Imu;
using JointStateMsg = sensor_msgs::msg::JointState;
using ContactMsg = state_estimator_msgs::msg::ContactDetection;
using AttitudeMsg = state_estimator_msgs::msg::Attitude;

using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, JointStateMsg, ContactMsg, AttitudeMsg>;
using ExactTimePolicy = message_filters::sync_policies::ExactTime<ImuMsg, JointStateMsg, ContactMsg, AttitudeMsg>;

#define MySyncPolicy ApproximateTimePolicy

//  This class calculates the base velocity as estimated from the leg kinematics. These are NOT the velocity of the legs.
	class LegOdometryPlugin : public PluginBase
	{
	public:
        LegOdometryPlugin() = default;

        ~LegOdometryPlugin() override = default;

		std::string getName() override { return std::string("LegOdometry"); }
		std::string getDescription() override { return std::string("Leg Odometry Plugin"); }

		void initialize_() override {

            auto node = this->node_;
            if (!node) {
                throw std::runtime_error("LegOdometryPlugin: node_ is null");
            }

            // URDF path param (ROS 2 style). Accept legacy name as fallback.
            std::string urdf_path_param = node->declare_parameter<std::string>(
                "leg_odometry_plugin.urdf_path", "");
            const std::string legacy_urdf = node->declare_parameter<std::string>(
                "leg_odometry_plugin/urdf_path", "");
            if (!legacy_urdf.empty()) {
                RCLCPP_WARN(node->get_logger(),
                    "Using legacy param 'leg_odometry_plugin/urdf_path'; prefer 'leg_odometry_plugin.urdf_path'");
                urdf_path_param = legacy_urdf;
            }

            if (urdf_path_param.empty()) {
                RCLCPP_ERROR(node->get_logger(), "URDF path is not set.");
                return;
            }

            // Resolve $(find <pkg>)/path using ament_index
            std::string resolved_path = urdf_path_param;
            const std::string find_token = "$(find ";
            if (resolved_path.find(find_token) != std::string::npos) {
                const size_t start = resolved_path.find(find_token) + find_token.length();
                const size_t end = resolved_path.find(')', start);
                const std::string package_name = resolved_path.substr(start, end - start);
                const std::string pkg_share = ament_index_cpp::get_package_share_directory(package_name);
                // Replace "$(find pkg)" with pkg_share
                resolved_path.replace(resolved_path.find(find_token), end - resolved_path.find(find_token) + 1, pkg_share);
            }

            RCLCPP_INFO(node->get_logger(), "Loading URDF from: %s", resolved_path.c_str());

            try {
                pinocchio::urdf::buildModel(resolved_path, model_);
                data_ = pinocchio::Data(model_);
                RCLCPP_INFO(node->get_logger(), "URDF loaded into Pinocchio model successfully.");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Failed to load URDF: %s", e.what());
            }

            // base_R_imu from attitude plugin params
            std::vector<double> base_R_imu_vec = node->declare_parameter<std::vector<double>>( 
                "leg_odometry_plugin.base_R_imu", std::vector<double>());
            if (base_R_imu_vec.size() == 9) {
                base_R_imu_ = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(base_R_imu_vec.data());
            } else {
                RCLCPP_WARN(node->get_logger(), "base_R_imu size != 9; using identity");
                base_R_imu_.setIdentity();
            }

            // Topics
            std::string imu_topic = node->declare_parameter<std::string>(
                "leg_odometry_plugin.imu_topic", "/sensors/imu");
            std::string joint_states_topic = node->declare_parameter<std::string>(
                "leg_odometry_plugin.joint_states_topic", "/state_estimator/joint_states");
            std::string contact_topic = node->declare_parameter<std::string>(
                "leg_odometry_plugin.contact_topic", "/state_estimator/contact_detection");
            std::string attitude_topic = node->declare_parameter<std::string>(
                "leg_odometry_plugin.attitude_topic", "/state_estimator/attitude");

            // Legacy ROS 1-style names (optional support)
            const std::string imu_topic_legacy = node->declare_parameter<std::string>("leg_odometry_plugin/imu_topic", "");
            const std::string js_topic_legacy  = node->declare_parameter<std::string>("leg_odometry_plugin/joint_states_topic", "");
            const std::string contact_topic_legacy = node->declare_parameter<std::string>("leg_odometry_plugin/contact_topic", "");
            const std::string attitude_topic_legacy = node->declare_parameter<std::string>("leg_odometry_plugin/attitude_topic", "");
            if (!imu_topic_legacy.empty()) imu_topic = imu_topic_legacy;
            if (!js_topic_legacy.empty()) joint_states_topic = js_topic_legacy;
            if (!contact_topic_legacy.empty()) contact_topic = contact_topic_legacy;
            if (!attitude_topic_legacy.empty()) attitude_topic = attitude_topic_legacy;

            // Subscribers with SensorDataQoS
            auto sensor_qos = rclcpp::SensorDataQoS();

            imu_sub_ = std::make_shared<message_filters::Subscriber<ImuMsg>>(node, imu_topic, sensor_qos);
            joint_state_sub_ = std::make_shared<message_filters::Subscriber<JointStateMsg>>(node, joint_states_topic, sensor_qos);
            contact_sub_ = std::make_shared<message_filters::Subscriber<ContactMsg>>(node, contact_topic, sensor_qos);
            attitude_sub_ = std::make_shared<message_filters::Subscriber<AttitudeMsg>>(node, attitude_topic, sensor_qos);

            sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(100), *imu_sub_, *joint_state_sub_, *contact_sub_, *attitude_sub_);
            sync_->registerCallback(std::bind(&LegOdometryPlugin::callback, this,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3,
                                              std::placeholders::_4));

            std::string pub_topic = node->declare_parameter<std::string>(
                "leg_odometry_plugin.pub_topic", "/state_estimator/leg_odometry");
            // Legacy
            const std::string pub_topic_legacy = node->declare_parameter<std::string>("leg_odometry_plugin/pub_topic", "");
            if (!pub_topic_legacy.empty()) {
                RCLCPP_WARN(node->get_logger(),
                  "Using legacy parameter 'leg_odometry_plugin/pub_topic'; prefer 'leg_odometry_plugin.pub_topic'");
                pub_topic = pub_topic_legacy;
            }
            pub_ = node->create_publisher<state_estimator_msgs::msg::LegOdometry>(pub_topic, rclcpp::QoS(10));

		}


		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

        void callback
        (
            const ImuMsg::ConstSharedPtr imu,
            const JointStateMsg::ConstSharedPtr js,
            const ContactMsg::ConstSharedPtr contact,
            const AttitudeMsg::ConstSharedPtr attitude
        )
		{
            std::cout << "LegOdometryPlugin callback called" << std::endl;

			// Robot joint states
            // Fill q and v from joint state
            Eigen::VectorXd q(model_.nq);
            Eigen::VectorXd v(model_.nv);

            if (js->position.size() != static_cast<size_t>(model_.nq) || js->velocity.size() != static_cast<size_t>(model_.nv)) {
                RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 1000,
                                     "Mismatch in joint state size (pos=%zu vs nq=%d, vel=%zu vs nv=%d)",
                                     js->position.size(), model_.nq, js->velocity.size(), model_.nv);
                return;
            }

            // Build map from joint name to position/velocity
            std::unordered_map<std::string, double> joint_pos_map, joint_vel_map;
            for (size_t i = 0; i < js->name.size(); ++i)
            {
                joint_pos_map[js->name[i]] = js->position[i];
                if (i < js->velocity.size())
                    joint_vel_map[js->name[i]] = js->velocity[i];
            }

            // Fill q and v in model's joint order (skip index 0 — "universe")
            for (pinocchio::JointIndex i = 1; i < model_.njoints; ++i)
            {
                const std::string& joint_name = model_.names[i];

                if (joint_pos_map.count(joint_name))
                    q[i - 1] = joint_pos_map[joint_name];
                else
                    q[i - 1] = 0.0;  // fallback

                if (joint_vel_map.count(joint_name))
                    v[i - 1] = joint_vel_map[joint_name];
                else
                    v[i - 1] = 0.0;
            }

           
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
			Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();

            base_velocity = w_R_b*base_velocity;

            // publishing	
            msg_.header.stamp = this->node_->get_clock()->now();

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
    
    std::shared_ptr<message_filters::Subscriber<ImuMsg>> imu_sub_;
    std::shared_ptr<message_filters::Subscriber<JointStateMsg>> joint_state_sub_;
    std::shared_ptr<message_filters::Subscriber<ContactMsg>> contact_sub_;
    std::shared_ptr<message_filters::Subscriber<AttitudeMsg>> attitude_sub_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

        rclcpp::Publisher<state_estimator_msgs::msg::LegOdometry>::SharedPtr pub_;

        state_estimator_msgs::msg::LegOdometry msg_;

        pinocchio::Model model_;
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
