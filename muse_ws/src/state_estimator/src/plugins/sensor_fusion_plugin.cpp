#include "state_estimator/Models/sensor_fusion.hpp"
#include "state_estimator/plugin.hpp"
#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "state_estimator/lib.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "unitree_go/msg/low_state.hpp"
#include "state_estimator_msgs/msg/leg_odometry.hpp"
#include "dls2_interface/msg/base_state.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <iit/commons/geometry/rotations.h>
#include <yaml-cpp/yaml.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <array>
#include <cmath>
#include <unordered_map>

namespace state_estimator_plugins
{

class SensorFusionPlugin : public PluginBase  
{
	public:
	SensorFusionPlugin() : attitude_(nullptr), sensor_fusion_(nullptr) {}
	~SensorFusionPlugin() { 
	if (attitude_ != nullptr) delete attitude_;
	if (sensor_fusion_ != nullptr) delete sensor_fusion_;
	}

	std::string getName() override { return std::string("SensorFusion"); }
	std::string getDescription() override { return std::string("Sensor Fusion Plugin"); }

	void initialize_() override 
	{
		t0_ = 0.0;
		xhat_estimated.setZero();
		xhat_att_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		xhat_att_.head(4).normalize();

		std::string low_state_topic = "/lowstate";
		std::string pub_topic       = "sensor_fusion";
		std::string base_state_topic = "/base_state";
		std::string robot_name = "go2";
		std::vector<double> P_vec(36, 0.0), Q_vec(36, 0.0), R_vec(9, 0.0);
		std::vector<double> base_R_imu_vec(9, 0.0);
		double ki_param = 0.02, kp_param = 10.0;
		std::vector<double> att_P_vec, att_Q_vec, att_R_vec;
		std::string urdf_path_param = "";
		contact_threshold_ = 5.0;

		// Default Go1 motor order for leg odometry computation
		motor_joint_names_ = {
			"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
			"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
			"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
			"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
		};
		feet_frame_names = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};

		if (!config_dir_.empty()) {
		try {
		YAML::Node sf_cfg = YAML::LoadFile(config_dir_ + "/sensor_fusion.yaml")["sensor_fusion_plugin"];
		if (sf_cfg["low_state_topic"])    low_state_topic = sf_cfg["low_state_topic"].as<std::string>();
		if (sf_cfg["pub_topic"])          pub_topic      = sf_cfg["pub_topic"].as<std::string>();
		if (sf_cfg["base_state_topic"])   base_state_topic = sf_cfg["base_state_topic"].as<std::string>();
		if (sf_cfg["robot_name"])         robot_name = sf_cfg["robot_name"].as<std::string>();
		if (sf_cfg["P"])  P_vec = sf_cfg["P"].as<std::vector<double>>();
		if (sf_cfg["Q"])  Q_vec = sf_cfg["Q"].as<std::vector<double>>();
		if (sf_cfg["R"])  R_vec = sf_cfg["R"].as<std::vector<double>>();

		YAML::Node att_cfg = YAML::LoadFile(config_dir_ + "/attitude_plugin.yaml")["attitude_estimation_plugin"];
		if (att_cfg["base_R_imu"])    base_R_imu_vec = att_cfg["base_R_imu"].as<std::vector<double>>();
		if (att_cfg["ki"])            ki_param         = att_cfg["ki"].as<double>();
		if (att_cfg["kp"])            kp_param         = att_cfg["kp"].as<double>();
		if (att_cfg["P"])             att_P_vec        = att_cfg["P"].as<std::vector<double>>();
		if (att_cfg["Q"])             att_Q_vec        = att_cfg["Q"].as<std::vector<double>>();
		if (att_cfg["R"])             att_R_vec        = att_cfg["R"].as<std::vector<double>>();
		if (att_cfg["gravity_vector"])gravity_vec_     = att_cfg["gravity_vector"].as<std::vector<double>>();
		if (att_cfg["north_vector"])  north_vec_       = att_cfg["north_vector"].as<std::vector<double>>();

		YAML::Node lo_cfg = YAML::LoadFile(config_dir_ + "/leg_odometry.yaml")["leg_odometry_plugin"];
		if (lo_cfg["urdf_path"])          urdf_path_param   = lo_cfg["urdf_path"].as<std::string>();
		if (lo_cfg["contact_force_threshold"]) contact_threshold_ = lo_cfg["contact_force_threshold"].as<double>();
		if (lo_cfg["motor_joint_names"])  motor_joint_names_ = lo_cfg["motor_joint_names"].as<std::vector<std::string>>();
		if (lo_cfg["feet_frame_names"])   feet_frame_names = lo_cfg["feet_frame_names"].as<std::vector<std::string>>();
		} catch (const std::exception& e) {
		RCLCPP_WARN(node_->get_logger(), "Could not load sensor fusion/leg odometry config: %s", e.what());
		}
		}

		if (P_vec.size() != 36 || Q_vec.size() != 36 || R_vec.size() != 9) {
		RCLCPP_WARN(node_->get_logger(), "P/Q/R have wrong size — using zeros.");
		P_vec.assign(36, 0.0); Q_vec.assign(36, 0.0); R_vec.assign(9, 0.0);
		}

		Eigen::Matrix6d P = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(P_vec.data());
		Eigen::Matrix6d Q = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(Q_vec.data());
		Eigen::Matrix3d R_sf = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(R_vec.data());

		sensor_fusion_ = new state_estimator::KFSensorFusion(t0_, xhat_estimated, P, Q, R_sf, false, false);

		if (base_R_imu_vec.size() == 9)
		base_R_imu_ = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(base_R_imu_vec.data());
		else
		base_R_imu_ = Eigen::Matrix3d::Identity();

		if (gravity_vec_.size() == 3)
		f_n_ = Eigen::Map<const Eigen::Vector3d>(gravity_vec_.data());
		else
		f_n_ = Eigen::Vector3d(0.0, 0.0, 9.81);
		if (north_vec_.size() == 3)
		m_n_ = Eigen::Map<const Eigen::Vector3d>(north_vec_.data());
		else
		m_n_ = Eigen::Vector3d(1./sqrt(3), 1./sqrt(3), 1./sqrt(3));

		// Initialize internal attitude estimator
		if (att_P_vec.size() == 36 && att_Q_vec.size() == 36 && att_R_vec.size() == 36) {
			Eigen::Matrix6d P0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(att_P_vec.data());
			Eigen::Matrix6d Q0  = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(att_Q_vec.data());
			Eigen::Matrix6d R0  = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(att_R_vec.data());
			attitude_ = new state_estimator::AttitudeBiasXKF(t0_, xhat_att_, P0, Q0, R0, f_n_, m_n_, ki_param, kp_param);
		} else {
			Eigen::Matrix6d P0 = 1e-6 * Eigen::Matrix6d::Identity();
			Eigen::Matrix6d Q0  = 1e-6 * Eigen::Matrix6d::Identity();
			Eigen::Matrix6d R0  = 1e-2 * Eigen::Matrix6d::Identity();
			attitude_ = new state_estimator::AttitudeBiasXKF(t0_, xhat_att_, P0, Q0, R0, f_n_, m_n_, ki_param, kp_param);
		}

		// Initialize Pinocchio model for leg odometry computation
		if (!urdf_path_param.empty()) {
		// Resolve $(find <pkg>) token
		const std::string find_token = "$(find ";
		if (urdf_path_param.find(find_token) != std::string::npos) {
		size_t s = urdf_path_param.find(find_token) + find_token.length();
		size_t e = urdf_path_param.find(")", s);
		std::string pkg = urdf_path_param.substr(s, e - s);
		try {
		std::string share = ament_index_cpp::get_package_share_directory(pkg);
		urdf_path_param.replace(urdf_path_param.find(find_token), e - urdf_path_param.find(find_token) + 1, share);
		} catch (...) {
		RCLCPP_ERROR(node_->get_logger(), "Cannot resolve package '%s' for URDF path", pkg.c_str());
		}
		}

		RCLCPP_INFO(node_->get_logger(), "Loading URDF from: %s", urdf_path_param.c_str());
		try {
		pinocchio::urdf::buildModel(urdf_path_param, model_);
		data_ = pinocchio::Data(model_);
		RCLCPP_INFO(node_->get_logger(), "URDF loaded for leg odometry computation.");
		} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to load URDF: %s", e.what());
		}
		}

		low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
		low_state_topic, 250,
		std::bind(&SensorFusionPlugin::callback_lowstate, this, std::placeholders::_1));

		pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(pub_topic, 250);
		base_state_pub_ = node_->create_publisher<dls2_interface::msg::BaseState>(base_state_topic, 250);
		base_state_msg_.robot_name = robot_name;
		base_state_msg_.stance_status.resize(4, false);
		RCLCPP_INFO(node_->get_logger(), "SensorFusionPlugin initialized on topic '%s'", low_state_topic.c_str());
	}

	void shutdown_() override { }
	void pause_() override { }
	void resume_() override { }
	void reset_() override { }

	void callback_lowstate(const unitree_go::msg::LowState::SharedPtr low_state)
	{
		auto finite3 = [](const Eigen::Vector3d& v) { return v.allFinite(); };
		auto finite6 = [](const Eigen::Matrix<double,6,1>& v) { return v.allFinite(); };
		auto finite7 = [](const Eigen::Vector7d& v) { return v.allFinite(); };

		if (begin_) {
			time_begin_ = node_->now().seconds();
			begin_ = false;
		}

		time_ = node_->now().seconds() - time_begin_;

		Eigen::Vector3d omega(
			low_state->imu_state.gyroscope[0],
			low_state->imu_state.gyroscope[1],
			low_state->imu_state.gyroscope[2]);
		Eigen::Vector3d acc(
			low_state->imu_state.accelerometer[0],
			low_state->imu_state.accelerometer[1],
			low_state->imu_state.accelerometer[2]);

		if (!finite3(omega) || !finite3(acc)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"SensorFusion: non-finite IMU data, skipping update");
			return;
		}

		Eigen::Vector3d f_b = base_R_imu_ * acc;
		Eigen::Vector3d omega_b = base_R_imu_ * omega;
		if (!finite3(f_b) || !finite3(omega_b)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"SensorFusion: non-finite transformed IMU data, skipping update");
			return;
		}

		if (!finite7(xhat_att_)) {
			xhat_att_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		}
		Eigen::Quaterniond quat_est;
		quat_est.w() = xhat_att_(0);
		quat_est.vec() << xhat_att_(1), xhat_att_(2), xhat_att_(3);
		Eigen::Vector3d m_b = iit::commons::quatToRotMat(quat_est) * m_n_;
		Eigen::Matrix<double,6,1> z_att;
		z_att << f_b, m_b;
		attitude_->update(time_, omega_b, z_att);
		Eigen::Vector7d xhat_att_new = attitude_->getX();
		if (finite7(xhat_att_new)) {
			xhat_att_ = xhat_att_new;
		} else {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"SensorFusion: attitude estimator returned NaN, keeping previous attitude");
		}

		double qnorm = xhat_att_.head(4).norm();
		if (!std::isfinite(qnorm) || qnorm < 1e-9) {
			xhat_att_.head(4) << 1.0, 0.0, 0.0, 0.0;
		} else {
			xhat_att_.head(4) /= qnorm;
		}

		// Extract updated quaternion and rotation matrix
		quat_est.w() = xhat_att_(0);
		quat_est.vec() << xhat_att_(1), xhat_att_(2), xhat_att_(3);
		Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();
		if (!w_R_b.allFinite()) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"SensorFusion: non-finite body rotation, using identity");
			w_R_b = Eigen::Matrix3d::Identity();
		}




		// ────────────── Compute leg odometry velocity v_b ──────────────
		Eigen::Vector3d v_b(0.0, 0.0, 0.0);
		bool stance_lf = false, stance_rf = false, stance_lh = false, stance_rh = false;

		// Only compute if model is loaded
		if (model_.nq > 0) {
		Eigen::Vector3d base_omega = base_R_imu_ * omega;

		// Build joint position and velocity maps from motor states
		std::unordered_map<std::string, double> joint_pos_map, joint_vel_map;
		const size_t n_motors = std::min(motor_joint_names_.size(),
										low_state->motor_state.size());
		for (size_t i = 0; i < n_motors; ++i) {
		joint_pos_map[motor_joint_names_[i]] = static_cast<double>(low_state->motor_state[i].q);
		joint_vel_map[motor_joint_names_[i]] = static_cast<double>(low_state->motor_state[i].dq);
		}

		// Fill q and v in model's joint order (skip index 0 — "universe")
		Eigen::VectorXd q(model_.nq), v(model_.nv);
		q.setZero(); v.setZero();
		for (pinocchio::JointIndex i = 1; i < model_.njoints; ++i) {
			const std::string& jname = model_.names[i];
			if (joint_pos_map.count(jname)) q[i - 1] = joint_pos_map[jname];
			if (joint_vel_map.count(jname)) v[i - 1] = joint_vel_map[jname];
		}

		// Contact detection: threshold foot_force
		stance_lf = std::abs(low_state->foot_force[0]) > contact_threshold_;
		stance_rf = std::abs(low_state->foot_force[1]) > contact_threshold_;
		stance_lh = std::abs(low_state->foot_force[2]) > contact_threshold_;
		stance_rh = std::abs(low_state->foot_force[3]) > contact_threshold_;

		// Compute forward kinematics
		pinocchio::forwardKinematics(model_, data_, q, v);
		pinocchio::updateFramePlacements(model_, data_);

		std::array<bool, 4> stance = {stance_lf, stance_rf, stance_lh, stance_rh};
		Eigen::Vector3d stance_vel_sum = Eigen::Vector3d::Zero();
		double stance_count = 0.0;
		for (size_t i = 0; i < feet_frame_names.size(); ++i) {
		const auto& foot_name = feet_frame_names[i];
		std::size_t frame_id = model_.getFrameId(foot_name);
		if (frame_id >= model_.nframes) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"SensorFusion: foot frame '%s' not found in model", foot_name.c_str());
			continue;
		}

		// With a fixed-base model, WORLD quantities are expressed in the base frame.
		pinocchio::Motion foot_vel_base = pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
		Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();

		// For a stance foot: 0 = v_b + ω×r + v_foot/base  ⇒  v_b = -(v_foot/base + ω×r)
		Eigen::Vector3d rel_vel = -(foot_vel_base.linear() + base_omega.cross(foot_pos_base));
		if (!finite3(rel_vel)) {
			continue;
		}
		if (stance[i]) {
			stance_vel_sum += rel_vel;
			stance_count += 1.0;
		}
		}

		if (stance_count > 0.5) {
			v_b = w_R_b * (stance_vel_sum / stance_count);
			if (!finite3(v_b)) {
				v_b.setZero();
			}
		}
		// std::cout << "v_b: " << v_b.transpose() << " stance_count: " << stance_count << std::endl;
		}

		// ────────────── Sensor fusion update ──────────────
		// Acceleration (world frame)
		Eigen::Vector3d w_f_b = w_R_b * f_b;
		Eigen::Vector3d gravity; gravity << 0.0, 0.0, -9.81;
		Eigen::Vector3d u_SF = w_f_b + gravity;

		// Use leg odometry velocity estimate
		Eigen::Vector3d z_SF = v_b;
		if (!finite3(u_SF) || !finite3(z_SF)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"SensorFusion: non-finite filter input, skipping KF update");
			u_SF.setZero();
			z_SF.setZero();
		}

		sensor_fusion_->predict(time_, u_SF);
		sensor_fusion_->update(time_, z_SF);
		Eigen::Matrix<double,6,1> xhat_new = sensor_fusion_->getX();
		if (finite6(xhat_new)) {
			xhat_estimated = xhat_new;
		} else {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"SensorFusion: KF state is NaN, publishing zeros as fallback");
			xhat_estimated.setZero();
		}

		// Publish results
		msg_.header.stamp = node_->now();
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

		// Eigen::Vector3d w_omega = w_R_b * omega;
		// if (!finite3(w_omega)) {
		// 	w_omega.setZero();
		// }

		Eigen::Vector7d xdot = attitude_->calc_f(time_, xhat_att_, omega_b);
		Eigen::Quaterniond quat_dot;
		quat_dot.w() = xdot(0);
		quat_dot.vec() << xdot(1), xdot(2), xdot(3);
		Eigen::Vector3d omega_filt = iit::commons::quatToOmega(quat_est, quat_dot);
		if (!finite3(omega_filt)) {
			omega_filt.setZero();
		}

		msg_.twist.twist.angular.x = omega_filt[0];
		msg_.twist.twist.angular.y = omega_filt[1];
		msg_.twist.twist.angular.z = omega_filt[2];

		pub_->publish(msg_);

		base_state_msg_.frame_id = "world";
		base_state_msg_.sequence_id = sequence_id_++;
		base_state_msg_.timestamp = static_cast<double>(node_->now().nanoseconds());
		base_state_msg_.pose.position[0] = msg_.pose.pose.position.x;
		base_state_msg_.pose.position[1] = msg_.pose.pose.position.y;
		base_state_msg_.pose.position[2] = msg_.pose.pose.position.z;
		base_state_msg_.pose.orientation[0] = msg_.pose.pose.orientation.x;
		base_state_msg_.pose.orientation[1] = msg_.pose.pose.orientation.y;
		base_state_msg_.pose.orientation[2] = msg_.pose.pose.orientation.z;
		base_state_msg_.pose.orientation[3] = msg_.pose.pose.orientation.w;
		base_state_msg_.velocity.linear[0] = msg_.twist.twist.linear.x;
		base_state_msg_.velocity.linear[1] = msg_.twist.twist.linear.y;
		base_state_msg_.velocity.linear[2] = msg_.twist.twist.linear.z;
		base_state_msg_.velocity.angular[0] = msg_.twist.twist.angular.x;
		base_state_msg_.velocity.angular[1] = msg_.twist.twist.angular.y;
		base_state_msg_.velocity.angular[2] = msg_.twist.twist.angular.z;
		base_state_msg_.acceleration.linear[0] = u_SF[0];
		base_state_msg_.acceleration.linear[1] = u_SF[1];
		base_state_msg_.acceleration.linear[2] = u_SF[2];
		base_state_msg_.acceleration.angular[0] = 0.0;
		base_state_msg_.acceleration.angular[1] = 0.0;
		base_state_msg_.acceleration.angular[2] = 0.0;
		base_state_msg_.stance_status[0] = stance_lf;
		base_state_msg_.stance_status[1] = stance_rf;
		base_state_msg_.stance_status[2] = stance_lh;
		base_state_msg_.stance_status[3] = stance_rh;
		base_state_pub_->publish(base_state_msg_);
	}

private:
std::shared_ptr<rclcpp::Subscription<unitree_go::msg::LowState>> low_state_sub_;
std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_;
std::shared_ptr<rclcpp::Publisher<dls2_interface::msg::BaseState>> base_state_pub_;

nav_msgs::msg::Odometry msg_;
dls2_interface::msg::BaseState base_state_msg_;

state_estimator::AttitudeBiasXKF* attitude_{nullptr};
state_estimator::KFSensorFusion* sensor_fusion_{nullptr};

Eigen::Vector7d xhat_att_;
Eigen::Matrix<double,6,1> xhat_estimated;
Eigen::Vector3d f_n_, m_n_;

double t0_{0.0};
double time_{0.0};
bool begin_{true};
double time_begin_{0.0};
std::vector<double> gravity_vec_, north_vec_;

Eigen::Matrix3d base_R_imu_;

// Leg odometry members
pinocchio::Model model_;
pinocchio::Data data_;
std::vector<std::string> motor_joint_names_;
std::vector<std::string> feet_frame_names;
double contact_threshold_{5.0};
uint32_t sequence_id_{0};
};

} // end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::SensorFusionPlugin, state_estimator_plugins::PluginBase)
