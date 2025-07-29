#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <iit/commons/geometry/rotations.h>

#include "state_estimator/plugin.hpp"
#include <ros/ros.h>
#include <ros/package.h>

#include "state_estimator_msgs/LegOdometry.h"
#include "state_estimator_msgs/JointStateWithAcceleration.h" 
#include "state_estimator_msgs/ContactDetection.h"
#include "state_estimator_msgs/attitude.h"

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <unordered_map>


namespace state_estimator_plugins
{

typedef message_filters::sync_policies::ApproximateTime
<
	sensor_msgs::Imu,
	// state_estimator_msgs::JointStateWithAcceleration,
    sensor_msgs::JointState,
    state_estimator_msgs::ContactDetection,
    state_estimator_msgs::attitude
> 
ApproximateTimePolicy;

typedef message_filters::sync_policies::ExactTime
<
	sensor_msgs::Imu,
	// state_estimator_msgs::JointStateWithAcceleration,
    sensor_msgs::JointState,
    state_estimator_msgs::ContactDetection,
    state_estimator_msgs::attitude
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
            attitude_sub(nullptr),
			pub_(nullptr), 
			sync_(nullptr) 
		{ } 
	
		~LegOdometryPlugin() 
		{
			// if (lo_!=nullptr) delete(lo_);
			if (imu_sub_!=nullptr) delete(imu_sub_);
			if (joint_state_sub_!=nullptr) delete(joint_state_sub_);
            if (contact_sub_!=nullptr) delete(contact_sub_);
            if (attitude_sub!=nullptr) delete(attitude_sub);
			if (pub_!=nullptr) delete(pub_);
			if (sync_!=nullptr) delete(sync_);
		}

		std::string getName() override { return std::string("LegOdometry"); }
		std::string getDescription() override { return std::string("Leg Odometry Plugin"); }

		void initialize_() override {

            std::string urdf_path_param;
            nh_.param("leg_odometry_plugin/urdf_path", urdf_path_param, std::string(""));

            if (urdf_path_param.empty()) {
                ROS_ERROR("URDF path is not set in parameter server.");
                return;
            }

            // Resolve $(find ...) manually if present
            std::string resolved_path = urdf_path_param;
            std::string find_token = "$(find ";
            if (resolved_path.find(find_token) != std::string::npos) {
                size_t start = resolved_path.find(find_token) + find_token.length();
                size_t end = resolved_path.find(")", start);
                std::string package_name = resolved_path.substr(start, end - start);
                std::string package_path = ros::package::getPath(package_name);
                resolved_path.replace(resolved_path.find(find_token), end - resolved_path.find(find_token) + 1, package_path);
            }

            ROS_INFO_STREAM("Loading URDF from: " << resolved_path);

            try {
                pinocchio::urdf::buildModel(resolved_path, model_);
                data_ = pinocchio::Data(model_);
                ROS_INFO("URDF loaded into Pinocchio model successfully.");
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("Failed to load URDF: " << e.what());
            }

            std::vector<double> base_R_imu_vec;
            nh_.param("attitude_estimation_plugin/base_R_imu", base_R_imu_vec, std::vector<double>(9, 0.0));

            if (base_R_imu_vec.size() == 9) {
                base_R_imu_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(base_R_imu_vec.data());
                // ROS_INFO_STREAM("Loaded base_R_imu:\n" << base_R_imu_);
            } else {
                ROS_WARN("base_R_imu is not the correct size (should be 9). Using identity.");
                base_R_imu_ = Eigen::Matrix3d::Identity();
            }

            std::string imu_topic, joint_states_topic, contact_topic, attitude_topic, pub_topic;
            
            // Get topic names from the parameter server
            nh_.param("leg_odometry_plugin/imu_topic", imu_topic, std::string("/sensors/imu"));
            nh_.param("leg_odometry_plugin/joint_states_topic", joint_states_topic, std::string("/state_estimator/joint_states"));
            nh_.param("leg_odometry_plugin/contact_topic", contact_topic, std::string("/state_estimator/contact_detection"));
            nh_.param("leg_odometry_plugin/attitude_topic", attitude_topic, std::string("/state_estimator/attitude"));
            
            // Set up subscribers using the loaded topic names
            imu_sub_ = new message_filters::Subscriber<sensor_msgs::Imu>(nh_, imu_topic, 250);
            // joint_state_sub_ = new message_filters::Subscriber<state_estimator_msgs::JointStateWithAcceleration>(nh_, joint_states_topic, 250);
            joint_state_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_, joint_states_topic, 250);
            contact_sub_ = new message_filters::Subscriber<state_estimator_msgs::ContactDetection>(nh_, contact_topic, 250);
            attitude_sub = new message_filters::Subscriber<state_estimator_msgs::attitude>(nh_, attitude_topic, 250);

			sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100),*imu_sub_,*joint_state_sub_,*contact_sub_,*attitude_sub); 
  			sync_->registerCallback(boost::bind(&LegOdometryPlugin::callback, this, _1, _2, _3,_4));

            nh_.param("leg_odometry_plugin/pub_topic", pub_topic, std::string("/state_estimator/leg_odometry"));
            pub_ = new ros::Publisher(nh_.advertise<state_estimator_msgs::LegOdometry>(pub_topic, 250));

		}


		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback
		(
			const sensor_msgs::Imu::ConstPtr& imu,
			// const state_estimator_msgs::JointStateWithAcceleration::ConstPtr& js,
            const sensor_msgs::JointState::ConstPtr& js,
            const state_estimator_msgs::ContactDetection::ConstPtr& contact,
            const state_estimator_msgs::attitude::ConstPtr& attitude
		)
		{
            std::cout << "LegOdometryPlugin callback called" << std::endl;

			// Robot joint states
            // Fill q and v from joint state
            Eigen::VectorXd q(model_.nq);
            Eigen::VectorXd v(model_.nv);

            if (js->position.size() != model_.nq || js->velocity.size() != model_.nv) {
            	ROS_WARN_THROTTLE(1.0, "Mismatch in joint state size");
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
			msg_.header.stamp = ros::Time::now();

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
	
		message_filters::Subscriber<sensor_msgs::Imu> *imu_sub_;
		// message_filters::Subscriber<state_estimator_msgs::JointStateWithAcceleration> *joint_state_sub_;
        message_filters::Subscriber<sensor_msgs::JointState> *joint_state_sub_;
        message_filters::Subscriber<state_estimator_msgs::ContactDetection> *contact_sub_;
        message_filters::Subscriber<state_estimator_msgs::attitude> *attitude_sub;
		message_filters::Synchronizer<MySyncPolicy> *sync_;

		ros::Publisher *pub_;

		state_estimator_msgs::LegOdometry msg_;

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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::LegOdometryPlugin, state_estimator_plugins::PluginBase)
