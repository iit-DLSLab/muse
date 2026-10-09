#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <iit/commons/geometry/rotations.h>

#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "state_estimator/Models/contact_estimator.hpp"
#include "state_estimator/Models/robot_config.hpp"
#include "state_estimator/Models/unitree_low_state_adapter.hpp"
#include "state_estimator/lib.hpp"
#include "state_estimator/plugin.hpp"

#include <rclcpp/rclcpp.hpp>

#include "state_estimator_msgs/msg/leg_odometry.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class LegOdometryPlugin : public PluginBase
{
public:
	LegOdometryPlugin() : attitude_(nullptr) {}
	~LegOdometryPlugin() override { if (attitude_ != nullptr) delete attitude_; }

	std::string getName() override { return std::string("LegOdometry"); }
	std::string getDescription() override { return std::string("Leg Odometry Plugin"); }

	void initialize_() override
	{
		t0_ = 0.0;
		state_estimator::RobotConfig robot_config;
		std::string config_error;
		if (!state_estimator::loadRobotConfig(config_dir_, robot_config, config_error)) {
			throw std::runtime_error("LegOdometry: " + config_error);
		}

		motor_joint_names_ = robot_config.motor_joint_names;
		motor_indices_ = robot_config.motor_indices;
		feet_frame_names_ = robot_config.feet_frame_names;
		std::string pub_topic = "/state_estimator/leg_odometry";
		std::vector<double> base_R_imu_vec(9, 0.0);
		double ki_param = 0.02, kp_param = 10.0;
		std::vector<double> P_vec, Q_vec, R_vec;

		if (!config_dir_.empty()) {
			try {
				YAML::Node lo_cfg = YAML::LoadFile(config_dir_ + "/leg_odometry.yaml")["leg_odometry_plugin"];
				if (lo_cfg["pub_topic"]) pub_topic = lo_cfg["pub_topic"].as<std::string>();

				YAML::Node att_cfg = YAML::LoadFile(config_dir_ + "/attitude_plugin.yaml")["attitude_estimation_plugin"];
				if (att_cfg["base_R_imu"]) base_R_imu_vec = att_cfg["base_R_imu"].as<std::vector<double>>();
				if (att_cfg["ki"]) ki_param = att_cfg["ki"].as<double>();
				if (att_cfg["kp"]) kp_param = att_cfg["kp"].as<double>();
				if (att_cfg["P"]) P_vec = att_cfg["P"].as<std::vector<double>>();
				if (att_cfg["Q"]) Q_vec = att_cfg["Q"].as<std::vector<double>>();
				if (att_cfg["R"]) R_vec = att_cfg["R"].as<std::vector<double>>();
				if (att_cfg["gravity_vector"]) gravity_vec_ = att_cfg["gravity_vector"].as<std::vector<double>>();
				if (att_cfg["north_vector"]) north_vec_ = att_cfg["north_vector"].as<std::vector<double>>();
			} catch (const std::exception& e) {
				RCLCPP_WARN(node_->get_logger(), "Could not load leg odometry config: %s", e.what());
			}
		}

		RCLCPP_INFO(node_->get_logger(), "Loading URDF from: %s", robot_config.urdf_path.c_str());
		try {
			pinocchio::urdf::buildModel(robot_config.urdf_path, model_);
			data_ = pinocchio::Data(model_);
			RCLCPP_INFO(node_->get_logger(), "URDF loaded into Pinocchio model.");
		} catch (const std::exception& e) {
			throw std::runtime_error(std::string("LegOdometry: failed to load URDF: ") + e.what());
		}

		if (!contact_estimator_.configure(robot_config)) {
			throw std::runtime_error("LegOdometry: " + contact_estimator_.lastError());
		}

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

		xhat_att_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		xhat_att_.head(4).normalize();
		if (P_vec.size() == 36 && Q_vec.size() == 36 && R_vec.size() == 36) {
			Eigen::Matrix6d P0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(P_vec.data());
			Eigen::Matrix6d Q = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(Q_vec.data());
			Eigen::Matrix6d R = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(R_vec.data());
			attitude_ = new state_estimator::AttitudeBiasXKF(t0_, xhat_att_, P0, Q, R, f_n_, m_n_, ki_param, kp_param);
		} else {
			RCLCPP_WARN(node_->get_logger(), "Missing/invalid P/Q/R - using scaled identity matrices.");
			attitude_ = new state_estimator::AttitudeBiasXKF(
				t0_, xhat_att_,
				1e-6 * Eigen::Matrix6d::Identity(),
				1e-6 * Eigen::Matrix6d::Identity(),
				1e-2 * Eigen::Matrix6d::Identity(),
				f_n_, m_n_, ki_param, kp_param);
		}

		pub_ = node_->create_publisher<state_estimator_msgs::msg::LegOdometry>(pub_topic, 250);
		createLowStateSubscription(robot_config.robot_type, robot_config.low_state_topic);
		RCLCPP_INFO(node_->get_logger(),
			"LegOdometryPlugin: robot=%s, source=%s, threshold=%.2f",
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
				std::bind(&LegOdometryPlugin::callbackGo, this, std::placeholders::_1));
			return;
		}

		hg_low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
			low_state_topic, 250,
			std::bind(&LegOdometryPlugin::callbackHg, this, std::placeholders::_1));
	}

	void callbackGo(const unitree_go::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: failed to adapt unitree_go LowState: %s", error.c_str());
			return;
		}
		processLowState(normalized_low_state_);
	}

	void callbackHg(const unitree_hg::msg::LowState::SharedPtr low_state)
	{
		std::string error;
		if (!state_estimator::fromRos(*low_state, normalized_low_state_, &error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: failed to adapt unitree_hg LowState: %s", error.c_str());
			return;
		}
		processLowState(normalized_low_state_);
	}

	void processLowState(const state_estimator::UnitreeLowState& low_state)
	{
		if (begin_) {
			time_begin_ = node_->now().seconds();
			begin_ = false;
		}
		time_ = node_->now().seconds() - time_begin_;

		const Eigen::Vector3d omega = low_state.imu.gyroscope;
		const Eigen::Vector3d acc = low_state.imu.accelerometer;
		const Eigen::Vector3d f_b = base_R_imu_ * acc;
		const Eigen::Vector3d omega_b = base_R_imu_ * omega;

		Eigen::Quaterniond quat_est;
		quat_est.w() = xhat_att_(0);
		quat_est.vec() << xhat_att_(1), xhat_att_(2), xhat_att_(3);
		Eigen::Vector3d m_b = iit::commons::quatToRotMat(quat_est) * m_n_;
		Eigen::Matrix<double,6,1> z;
		z << f_b, m_b;
		attitude_->update(time_, omega_b, z);
		xhat_att_ = attitude_->getX();

		quat_est.w() = xhat_att_(0);
		quat_est.vec() << xhat_att_(1), xhat_att_(2), xhat_att_(3);
		Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();
		Eigen::Vector3d base_omega = base_R_imu_ * omega;

		Eigen::VectorXd q(model_.nq), v(model_.nv);
		std::string joint_state_error;
		if (!state_estimator::fillPinocchioStateFromLowState(
			model_, motor_joint_names_, motor_indices_, low_state, q, v, joint_state_error)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: failed to fill Pinocchio joint state: %s",
				joint_state_error.c_str());
			return;
		}

		std::array<bool, 4> stance = estimateStance(low_state);
		stance_lf = stance[0];
		stance_rf = stance[1];
		stance_lh = stance[2];
		stance_rh = stance[3];

		pinocchio::forwardKinematics(model_, data_, q, v);
		pinocchio::updateFramePlacements(model_, data_);

		std::vector<Eigen::Vector3d> foot_vels;
		foot_vels.reserve(4);
		for (const auto& foot_name : feet_frame_names_) {
			if (!model_.existFrame(foot_name)) {
				RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
					"LegOdometry: foot frame '%s' not found in model", foot_name.c_str());
				foot_vels.push_back(Eigen::Vector3d::Zero());
				continue;
			}

			pinocchio::FrameIndex frame_id = model_.getFrameId(foot_name);
			pinocchio::Motion foot_vel_base =
				pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
			Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();
			foot_vels.push_back(-(foot_vel_base.linear() + base_omega.cross(foot_pos_base)));
		}

		if (foot_vels.size() != 4) {
			return;
		}

		const Eigen::Vector3d lin_leg_lf = foot_vels[0];
		const Eigen::Vector3d lin_leg_rf = foot_vels[1];
		const Eigen::Vector3d lin_leg_lh = foot_vels[2];
		const Eigen::Vector3d lin_leg_rh = foot_vels[3];
		const double sum_stance = stance_lf + stance_rf + stance_lh + stance_rh;
		Eigen::Vector3d base_velocity =
			(stance_lf * lin_leg_lf + stance_rf * lin_leg_rf +
			 stance_lh * lin_leg_lh + stance_rh * lin_leg_rh) / (sum_stance + 1e-5);
		base_velocity = w_R_b * base_velocity;

		msg_.header.stamp = node_->now();
		for (int j = 0; j < 3; ++j) {
			msg_.lin_vel_lf[j] = lin_leg_lf.data()[j];
			msg_.lin_vel_rf[j] = lin_leg_rf.data()[j];
			msg_.lin_vel_lh[j] = lin_leg_lh.data()[j];
			msg_.lin_vel_rh[j] = lin_leg_rh.data()[j];
			msg_.base_velocity[j] = base_velocity.data()[j];
		}
		pub_->publish(msg_);
	}

	std::array<bool, 4> estimateStance(const state_estimator::UnitreeLowState& low_state)
	{
		state_estimator::ContactEstimate estimate;
		if (!contact_estimator_.update(low_state, estimate)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: contact update failed: %s",
				contact_estimator_.lastError().c_str());
			return estimate.stance;
		}

		if (!contact_estimator_.lastError().empty()) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: partial contact estimate: %s",
				contact_estimator_.lastError().c_str());
		}
		RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
			"LegOdometry contact metrics [FL FR RL RR]: %.3f %.3f %.3f %.3f; condition: %.2f %.2f %.2f %.2f",
			estimate.metric[0], estimate.metric[1], estimate.metric[2], estimate.metric[3],
			estimate.condition_number[0], estimate.condition_number[1],
			estimate.condition_number[2], estimate.condition_number[3]);
		return estimate.stance;
	}

	rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr go_low_state_sub_;
	rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr hg_low_state_sub_;
	rclcpp::Publisher<state_estimator_msgs::msg::LegOdometry>::SharedPtr pub_;

	state_estimator_msgs::msg::LegOdometry msg_;
	state_estimator::UnitreeLowState normalized_low_state_;
	state_estimator::AttitudeBiasXKF* attitude_{nullptr};
	state_estimator::ContactEstimator contact_estimator_;

	Eigen::Vector7d xhat_att_;
	Eigen::Vector3d f_n_, m_n_;
	Eigen::Matrix3d base_R_imu_;

	std::vector<std::string> motor_joint_names_;
	std::vector<std::size_t> motor_indices_;
	std::vector<std::string> feet_frame_names_;

	double t0_{0.0};
	double time_{0.0};
	bool begin_{true};
	double time_begin_{0.0};
	std::vector<double> gravity_vec_, north_vec_;

	pinocchio::Model model_;
	pinocchio::Data data_;

	bool stance_lf{false};
	bool stance_rf{false};
	bool stance_lh{false};
	bool stance_rh{false};
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::LegOdometryPlugin, state_estimator_plugins::PluginBase)
