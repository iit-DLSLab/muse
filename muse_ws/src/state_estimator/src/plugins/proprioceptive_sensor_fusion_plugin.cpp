#include <cstdint>

#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "state_estimator/Models/contact_estimator.hpp"
#include "state_estimator/Models/proprioceptive_sensor_fusion.hpp"
#include "state_estimator/Models/robot_config.hpp"
#include "state_estimator/Models/unitree_low_state_adapter.hpp"
#include "state_estimator/lib.hpp"
#include "state_estimator/plugin.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <iit/commons/geometry/rotations.h>

#include <rclcpp/rclcpp.hpp>

#include "dls2_interface/msg/base_state.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class ProprioceptiveSensorFusionPlugin : public PluginBase
{
public:
	ProprioceptiveSensorFusionPlugin() : attitude_(nullptr), proprioceptive_sensor_fusion_(nullptr) {}
	~ProprioceptiveSensorFusionPlugin() override
	{
		if (attitude_ != nullptr) delete attitude_;
		if (proprioceptive_sensor_fusion_ != nullptr) delete proprioceptive_sensor_fusion_;
	}

	std::string getName() override { return std::string("ProprioceptiveSensorFusion"); }
	std::string getDescription() override { return std::string("Proprioceptive Sensor Fusion Plugin"); }

	void initialize_() override
	{
		t0_ = 0.0;
		xhat_estimated.setZero();
		xhat_att_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		xhat_att_.head(4).normalize();

		state_estimator::RobotConfig robot_config;
		std::string config_error;
		if (!state_estimator::loadRobotConfig(config_dir_, robot_config, config_error)) {
			throw std::runtime_error("ProprioceptiveSensorFusion: " + config_error);
		}

		motor_joint_names_ = robot_config.motor_joint_names;
		motor_indices_ = robot_config.motor_indices;
		feet_frame_names_ = robot_config.feet_frame_names;
		std::string pub_topic = "proprioceptive_sensor_fusion";
		std::string base_state_topic = "/base_state";
		std::vector<double> P_vec(36, 0.0), Q_vec(36, 0.0), R_vec(9, 0.0);
		std::vector<double> base_R_imu_vec(9, 0.0);
		std::vector<double> att_P_vec, att_Q_vec, att_R_vec;
		double ki_param = 0.02, kp_param = 10.0;
		world_frame_id_ = "world";
		base_frame_id_ = "base";

		if (!config_dir_.empty()) {
			try {
				YAML::Node sf_cfg = YAML::LoadFile(config_dir_ + "/proprioceptive_sensor_fusion.yaml")["proprioceptive_sensor_fusion_plugin"];
				if (sf_cfg["pub_topic"]) pub_topic = sf_cfg["pub_topic"].as<std::string>();
				if (sf_cfg["base_state_topic"]) base_state_topic = sf_cfg["base_state_topic"].as<std::string>();
				if (sf_cfg["world_frame_id"]) world_frame_id_ = sf_cfg["world_frame_id"].as<std::string>();
				if (sf_cfg["base_frame_id"]) base_frame_id_ = sf_cfg["base_frame_id"].as<std::string>();
				if (sf_cfg["P"]) P_vec = sf_cfg["P"].as<std::vector<double>>();
				if (sf_cfg["Q"]) Q_vec = sf_cfg["Q"].as<std::vector<double>>();
				if (sf_cfg["R"]) R_vec = sf_cfg["R"].as<std::vector<double>>();

				YAML::Node att_cfg = YAML::LoadFile(config_dir_ + "/attitude_plugin.yaml")["attitude_estimation_plugin"];
				if (att_cfg["base_R_imu"]) base_R_imu_vec = att_cfg["base_R_imu"].as<std::vector<double>>();
				if (att_cfg["ki"]) ki_param = att_cfg["ki"].as<double>();
				if (att_cfg["kp"]) kp_param = att_cfg["kp"].as<double>();
				if (att_cfg["P"]) att_P_vec = att_cfg["P"].as<std::vector<double>>();
				if (att_cfg["Q"]) att_Q_vec = att_cfg["Q"].as<std::vector<double>>();
				if (att_cfg["R"]) att_R_vec = att_cfg["R"].as<std::vector<double>>();
				if (att_cfg["gravity_vector"]) gravity_vec_ = att_cfg["gravity_vector"].as<std::vector<double>>();
				if (att_cfg["north_vector"]) north_vec_ = att_cfg["north_vector"].as<std::vector<double>>();
			} catch (const std::exception& e) {
				RCLCPP_WARN(node_->get_logger(), "Could not load sensor fusion config: %s", e.what());
			}
		}

		if (P_vec.size() != 36 || Q_vec.size() != 36 || R_vec.size() != 9) {
			RCLCPP_WARN(node_->get_logger(), "P/Q/R have wrong size - using zeros.");
			P_vec.assign(36, 0.0);
			Q_vec.assign(36, 0.0);
			R_vec.assign(9, 0.0);
		}

		Eigen::Matrix6d P = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(P_vec.data());
		Eigen::Matrix6d Q = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(Q_vec.data());
		Eigen::Matrix3d R_sf = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(R_vec.data());
		proprioceptive_sensor_fusion_ =
			new state_estimator::KFProprioceptiveSensorFusion(t0_, xhat_estimated, P, Q, R_sf, false, false);

		if (base_R_imu_vec.size() == 9) {
			base_R_imu_ = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(base_R_imu_vec.data());
		} else {
			base_R_imu_ = Eigen::Matrix3d::Identity();
		}
		if (gravity_vec_.size() == 3) {
			f_n_ = Eigen::Map<const Eigen::Vector3d>(gravity_vec_.data());
		} else {
			f_n_ = Eigen::Vector3d(0.0, 0.0, 9.81);
		}
		if (north_vec_.size() == 3) {
			m_n_ = Eigen::Map<const Eigen::Vector3d>(north_vec_.data());
		} else {
			m_n_ = Eigen::Vector3d(1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0));
		}

		if (att_P_vec.size() == 36 && att_Q_vec.size() == 36 && att_R_vec.size() == 36) {
			Eigen::Matrix6d P0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(att_P_vec.data());
			Eigen::Matrix6d Q0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(att_Q_vec.data());
			Eigen::Matrix6d R0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(att_R_vec.data());
			attitude_ = new state_estimator::AttitudeBiasXKF(t0_, xhat_att_, P0, Q0, R0, f_n_, m_n_, ki_param, kp_param);
		} else {
			attitude_ = new state_estimator::AttitudeBiasXKF(
				t0_, xhat_att_,
				1e-6 * Eigen::Matrix6d::Identity(),
				1e-6 * Eigen::Matrix6d::Identity(),
				1e-2 * Eigen::Matrix6d::Identity(),
				f_n_, m_n_, ki_param, kp_param);
		}

		RCLCPP_INFO(node_->get_logger(), "Loading URDF from: %s", robot_config.urdf_path.c_str());
		try {
			pinocchio::urdf::buildModel(robot_config.urdf_path, model_);
			data_ = pinocchio::Data(model_);
			RCLCPP_INFO(node_->get_logger(), "URDF loaded for leg odometry computation.");
		} catch (const std::exception& e) {
			throw std::runtime_error(
				std::string("ProprioceptiveSensorFusion: failed to load URDF: ") + e.what());
		}

		if (!contact_estimator_.configure(robot_config)) {
			throw std::runtime_error(
				"ProprioceptiveSensorFusion: " + contact_estimator_.lastError());
		}

		pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(pub_topic, 250);
		base_state_pub_ = node_->create_publisher<dls2_interface::msg::BaseState>(base_state_topic, 250);
		base_state_msg_.robot_name = robot_config.robot_type;
		base_state_msg_.stance_status.resize(4, false);
		createLowStateSubscription(robot_config.robot_type, robot_config.low_state_topic);

		RCLCPP_INFO(node_->get_logger(),
			"ProprioceptiveSensorFusionPlugin: robot=%s, source=%s, threshold=%.2f",
			robot_config.robot_type.c_str(),
			contact_estimator_.usesRawFootForce() ? "LowState.foot_force" : "filtered Pinocchio GRF",
			robot_config.contact.threshold);
	}

	void shutdown_() override {}
	void pause_() override {}
	void resume_() override {}
	void reset_() override { contact_estimator_.reset(); }

private:
	void createLowStateSubscription(const std::string& robot_type, const std::string& low_state_topic)
	{
		const auto family = state_estimator::messageFamilyForRobotType(robot_type);
		if (family == state_estimator::UnitreeMessageFamily::Go) {
			go_low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
				low_state_topic, 250,
				std::bind(&ProprioceptiveSensorFusionPlugin::callbackGo, this, std::placeholders::_1));
			return;
		}

		hg_low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
			low_state_topic, 250,
			std::bind(&ProprioceptiveSensorFusionPlugin::callbackHg, this, std::placeholders::_1));
	}

	void callbackGo(const unitree_go::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ProprioceptiveSensorFusion: failed to adapt unitree_go LowState: %s", error.c_str());
			return;
		}
		processLowState(normalized_low_state_);
	}

	void callbackHg(const unitree_hg::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ProprioceptiveSensorFusion: failed to adapt unitree_hg LowState: %s", error.c_str());
			return;
		}
		processLowState(normalized_low_state_);
	}

	void processLowState(const state_estimator::UnitreeLowState& low_state)
	{
		auto finite3 = [](const Eigen::Vector3d& v) { return v.allFinite(); };
		auto finite6 = [](const Eigen::Matrix<double,6,1>& v) { return v.allFinite(); };
		auto finite7 = [](const Eigen::Vector7d& v) { return v.allFinite(); };

		if (begin_) {
			time_begin_ = node_->now().seconds();
			begin_ = false;
		}
		time_ = node_->now().seconds() - time_begin_;

		const Eigen::Vector3d omega = low_state.imu.gyroscope;
		const Eigen::Vector3d acc = low_state.imu.accelerometer;
		if (!finite3(omega) || !finite3(acc)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ProprioceptiveSensorFusion: non-finite IMU data, skipping update");
			return;
		}

		Eigen::Vector3d f_b = base_R_imu_ * acc;
		Eigen::Vector3d omega_b = base_R_imu_ * omega;
		if (!finite3(f_b) || !finite3(omega_b)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ProprioceptiveSensorFusion: non-finite transformed IMU data, skipping update");
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
		}

		const double qnorm = xhat_att_.head(4).norm();
		if (!std::isfinite(qnorm) || qnorm < 1e-9) {
			xhat_att_.head(4) << 1.0, 0.0, 0.0, 0.0;
		} else {
			xhat_att_.head(4) /= qnorm;
		}

		quat_est.w() = xhat_att_(0);
		quat_est.vec() << xhat_att_(1), xhat_att_(2), xhat_att_(3);
		Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();
		if (!w_R_b.allFinite()) {
			w_R_b = Eigen::Matrix3d::Identity();
		}

		Eigen::Vector3d v_b = Eigen::Vector3d::Zero();
		std::array<bool, 4> stance{{false, false, false, false}};

		if (model_.nq > 0) {
			Eigen::VectorXd q(model_.nq), v(model_.nv);
			std::string joint_state_error;
			if (!state_estimator::fillPinocchioStateFromLowState(
				model_, motor_joint_names_, motor_indices_, low_state, q, v, joint_state_error)) {
				RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
					"ProprioceptiveSensorFusion: failed to fill Pinocchio joint state: %s",
					joint_state_error.c_str());
			} else {
				stance = estimateStance(low_state);
				pinocchio::forwardKinematics(model_, data_, q, v);
				pinocchio::updateFramePlacements(model_, data_);

				const Eigen::Vector3d base_omega = base_R_imu_ * omega;
				Eigen::Vector3d stance_vel_sum = Eigen::Vector3d::Zero();
				double stance_count = 0.0;
				for (std::size_t i = 0; i < feet_frame_names_.size(); ++i) {
					const auto& foot_name = feet_frame_names_[i];
					if (!model_.existFrame(foot_name)) {
						RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
							"ProprioceptiveSensorFusion: foot frame '%s' not found in model", foot_name.c_str());
						continue;
					}
					const pinocchio::FrameIndex frame_id = model_.getFrameId(foot_name);
					const pinocchio::Motion foot_vel_base =
						pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
					const Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();
					const Eigen::Vector3d rel_vel = -(foot_vel_base.linear() + base_omega.cross(foot_pos_base));
					if (stance[i] && finite3(rel_vel)) {
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
			}
		}

		const Eigen::Vector3d w_f_b = w_R_b * f_b;
		const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
		Eigen::Vector3d u_SF = w_f_b + gravity;
		if (!finite3(u_SF)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ProprioceptiveSensorFusion: non-finite filter input, using zero acceleration");
			u_SF.setZero();
		}

		proprioceptive_sensor_fusion_->predict(time_, u_SF);
		proprioceptive_sensor_fusion_->update(time_, v_b);

		Eigen::Matrix<double,6,1> xhat_new = proprioceptive_sensor_fusion_->getX();
		xhat_estimated = finite6(xhat_new) ? xhat_new : Eigen::Matrix<double,6,1>::Zero();

		publishState(quat_est, omega_b, u_SF, stance);
	}

	std::array<bool, 4> estimateStance(const state_estimator::UnitreeLowState& low_state)
	{
		state_estimator::ContactEstimate estimate;
		if (!contact_estimator_.update(low_state, estimate)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ProprioceptiveSensorFusion: contact update failed: %s",
				contact_estimator_.lastError().c_str());
			return estimate.stance;
		}

		if (!contact_estimator_.lastError().empty()) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"ProprioceptiveSensorFusion: partial contact estimate: %s",
				contact_estimator_.lastError().c_str());
		}
		RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
			"ProprioceptiveSensorFusion contact metrics [FL FR RL RR]: %.3f %.3f %.3f %.3f; condition: %.2f %.2f %.2f %.2f",
			estimate.metric[0], estimate.metric[1], estimate.metric[2], estimate.metric[3],
			estimate.condition_number[0], estimate.condition_number[1],
			estimate.condition_number[2], estimate.condition_number[3]);
		return estimate.stance;
	}

	void publishState(
		const Eigen::Quaterniond& quat_est,
		const Eigen::Vector3d& omega_b,
		const Eigen::Vector3d& u_SF,
		const std::array<bool, 4>& stance)
	{
		msg_.header.stamp = node_->now();
		msg_.header.frame_id = world_frame_id_;
		msg_.child_frame_id = base_frame_id_;
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

		Eigen::Vector7d xdot = attitude_->calc_f(time_, xhat_att_, omega_b);
		Eigen::Quaterniond quat_dot;
		quat_dot.w() = xdot(0);
		quat_dot.vec() << xdot(1), xdot(2), xdot(3);
		Eigen::Vector3d omega_filt = iit::commons::quatToOmega(quat_est, quat_dot);
		if (!omega_filt.allFinite()) {
			omega_filt.setZero();
		}
		msg_.twist.twist.angular.x = omega_filt[0];
		msg_.twist.twist.angular.y = omega_filt[1];
		msg_.twist.twist.angular.z = omega_filt[2];
		pub_->publish(msg_);

		base_state_msg_.frame_id = world_frame_id_;
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
		base_state_msg_.stance_status[0] = stance[0];
		base_state_msg_.stance_status[1] = stance[1];
		base_state_msg_.stance_status[2] = stance[2];
		base_state_msg_.stance_status[3] = stance[3];
		base_state_pub_->publish(base_state_msg_);
	}

	rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr go_low_state_sub_;
	rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr hg_low_state_sub_;
	std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_;
	std::shared_ptr<rclcpp::Publisher<dls2_interface::msg::BaseState>> base_state_pub_;

	nav_msgs::msg::Odometry msg_;
	dls2_interface::msg::BaseState base_state_msg_;
	state_estimator::UnitreeLowState normalized_low_state_;
	state_estimator::AttitudeBiasXKF* attitude_{nullptr};
	state_estimator::KFProprioceptiveSensorFusion* proprioceptive_sensor_fusion_{nullptr};
	state_estimator::ContactEstimator contact_estimator_;

	Eigen::Vector7d xhat_att_;
	Eigen::Matrix<double,6,1> xhat_estimated;
	Eigen::Vector3d f_n_, m_n_;
	Eigen::Matrix3d base_R_imu_;

	pinocchio::Model model_;
	pinocchio::Data data_;
	std::vector<std::string> motor_joint_names_;
	std::vector<std::size_t> motor_indices_;
	std::vector<std::string> feet_frame_names_;

	double t0_{0.0};
	double time_{0.0};
	bool begin_{true};
	double time_begin_{0.0};
	std::vector<double> gravity_vec_, north_vec_;
	uint32_t sequence_id_{0};
	std::string world_frame_id_;
	std::string base_frame_id_;
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ProprioceptiveSensorFusionPlugin, state_estimator_plugins::PluginBase)
