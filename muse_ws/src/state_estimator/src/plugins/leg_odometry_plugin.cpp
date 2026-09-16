#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <iit/commons/geometry/rotations.h>

#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "state_estimator/Models/grf_estimator.hpp"
#include "state_estimator/Models/unitree_low_state_adapter.hpp"
#include "state_estimator/lib.hpp"
#include "state_estimator/plugin.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "state_estimator_msgs/msg/leg_odometry.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <exception>
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
		std::string robot_type = "go2";
		std::string urdf_path_param = "$(find state_estimator)/urdfs/go2.urdf";
		std::string low_state_topic = "/lowstate";
		std::string pub_topic = "/state_estimator/leg_odometry";
		std::vector<double> base_R_imu_vec(9, 0.0);
		double ki_param = 0.02, kp_param = 10.0;
		std::vector<double> P_vec, Q_vec, R_vec;
		std::vector<double> contact_normal_vec{0.0, 0.0, 1.0};
		double force_sign = -1.0;
		bool use_inverse_dynamics_compensation = true;
		contact_threshold_ = 15.0;
		contact_threshold_low_ = contact_threshold_;
		contact_threshold_high_ = contact_threshold_;

		motor_joint_names_ = defaultMotorJointNames();
		motor_indices_ = state_estimator::defaultMotorIndices(motor_joint_names_.size());
		feet_frame_names_ = defaultFeetFrameNames();

		if (!config_dir_.empty()) {
			try {
				YAML::Node lo_cfg = YAML::LoadFile(config_dir_ + "/leg_odometry.yaml")["leg_odometry_plugin"];
				if (lo_cfg["robot_type"]) robot_type = lo_cfg["robot_type"].as<std::string>();
				if (lo_cfg["urdf_path"]) urdf_path_param = lo_cfg["urdf_path"].as<std::string>();
				if (lo_cfg["low_state_topic"]) low_state_topic = lo_cfg["low_state_topic"].as<std::string>();
				if (lo_cfg["pub_topic"]) pub_topic = lo_cfg["pub_topic"].as<std::string>();
				if (lo_cfg["contact_force_threshold"]) contact_threshold_ = lo_cfg["contact_force_threshold"].as<double>();
				if (lo_cfg["grf_threshold"]) contact_threshold_ = lo_cfg["grf_threshold"].as<double>();
				contact_threshold_low_ = contact_threshold_;
				contact_threshold_high_ = contact_threshold_;
				if (lo_cfg["threshold_low"]) contact_threshold_low_ = lo_cfg["threshold_low"].as<double>();
				if (lo_cfg["threshold_high"]) contact_threshold_high_ = lo_cfg["threshold_high"].as<double>();
				if (lo_cfg["contact_force_threshold_low"]) {
					contact_threshold_low_ = lo_cfg["contact_force_threshold_low"].as<double>();
				}
				if (lo_cfg["contact_force_threshold_high"]) {
					contact_threshold_high_ = lo_cfg["contact_force_threshold_high"].as<double>();
				}
				if (lo_cfg["grf_threshold_low"]) contact_threshold_low_ = lo_cfg["grf_threshold_low"].as<double>();
				if (lo_cfg["grf_threshold_high"]) contact_threshold_high_ = lo_cfg["grf_threshold_high"].as<double>();
				if (lo_cfg["use_hysteresis"]) use_grf_hysteresis_ = lo_cfg["use_hysteresis"].as<bool>();
				if (lo_cfg["use_grf_hysteresis"]) use_grf_hysteresis_ = lo_cfg["use_grf_hysteresis"].as<bool>();
				if (lo_cfg["motor_joint_names"]) motor_joint_names_ = lo_cfg["motor_joint_names"].as<std::vector<std::string>>();
				if (lo_cfg["motor_indices"]) motor_indices_ = toSizeTVector(lo_cfg["motor_indices"].as<std::vector<int>>());
				if (lo_cfg["feet_frame_names"]) feet_frame_names_ = lo_cfg["feet_frame_names"].as<std::vector<std::string>>();
				if (lo_cfg["contact_normal"]) contact_normal_vec = lo_cfg["contact_normal"].as<std::vector<double>>();
				if (lo_cfg["use_force_norm"]) use_force_norm_ = lo_cfg["use_force_norm"].as<bool>();
				if (lo_cfg["use_absolute_normal_force"]) use_absolute_normal_force_ = lo_cfg["use_absolute_normal_force"].as<bool>();
				if (lo_cfg["force_sign"]) force_sign = lo_cfg["force_sign"].as<double>();
				if (lo_cfg["use_inverse_dynamics_compensation"]) {
					use_inverse_dynamics_compensation = lo_cfg["use_inverse_dynamics_compensation"].as<bool>();
				}

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

		validateConfiguredNamesAndIndices("LegOdometry");
		urdf_path_param = resolvePackagePath(urdf_path_param);

		RCLCPP_INFO(node_->get_logger(), "Loading URDF from: %s", urdf_path_param.c_str());
		try {
			pinocchio::urdf::buildModel(urdf_path_param, model_);
			data_ = pinocchio::Data(model_);
			RCLCPP_INFO(node_->get_logger(), "URDF loaded into Pinocchio model.");
		} catch (const std::exception& e) {
			RCLCPP_ERROR(node_->get_logger(), "Failed to load URDF: %s", e.what());
		}

		contact_normal_ = vector3FromConfig(contact_normal_vec, Eigen::Vector3d::UnitZ());

		state_estimator::GrfEstimatorOptions grf_options;
		grf_options.urdf_path = urdf_path_param;
		grf_options.motor_joint_names = motor_joint_names_;
		grf_options.motor_indices = motor_indices_;
		grf_options.feet_frame_names = feet_frame_names_;
		grf_options.force_sign = force_sign;
		grf_options.use_inverse_dynamics_compensation = use_inverse_dynamics_compensation;
		grf_estimator_configured_ = grf_estimator_.configure(grf_options);
		if (!grf_estimator_configured_) {
			RCLCPP_ERROR(node_->get_logger(),
				"LegOdometry: failed to configure GRF estimator: %s",
				grf_estimator_.lastError().c_str());
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
		createLowStateSubscription(robot_type, low_state_topic);
		RCLCPP_INFO(node_->get_logger(),
			"LegOdometryPlugin initialized on topic '%s' for robot_type '%s' with GRF threshold %.2f, hysteresis=%s [low=%.2f high=%.2f]",
			low_state_topic.c_str(), robot_type.c_str(), contact_threshold_,
			use_grf_hysteresis_ ? "true" : "false",
			contact_threshold_low_, contact_threshold_high_);
	}

	void shutdown_() override {}
	void pause_() override {}
	void resume_() override {}
	void reset_() override {}

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

		std::array<bool, 4> stance = estimateStanceFromGrf(low_state);
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

	std::array<bool, 4> estimateStanceFromGrf(const state_estimator::UnitreeLowState& low_state)
	{
		std::array<bool, 4> stance{{false, false, false, false}};
		std::array<double, 4> metrics{{0.0, 0.0, 0.0, 0.0}};

		if (!grf_estimator_configured_) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: GRF estimator is not configured; using no-contact stance");
			resetStanceState();
			return stance;
		}

		state_estimator::GrfEstimate estimate;
		if (!grf_estimator_.updateFromLowState(low_state, estimate)) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: GRF estimator update failed: %s",
				grf_estimator_.lastError().c_str());
			resetStanceState();
			return stance;
		}

		for (std::size_t i = 0; i < stance.size(); ++i) {
			if (!estimate.valid[i]) {
				stance_state_[i] = false;
				continue;
			}
			metrics[i] = state_estimator::grfContactMetric(
				estimate.force[i], contact_normal_, use_force_norm_, use_absolute_normal_force_);
			stance[i] = contactFromMetric(i, metrics[i]);
			stance_state_[i] = stance[i];
		}

		if (!grf_estimator_.lastError().empty()) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
				"LegOdometry: partial GRF estimate: %s", grf_estimator_.lastError().c_str());
		}
		RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
			"LegOdometry GRF metrics [FL FR RL RR]: %.3f %.3f %.3f %.3f",
			metrics[0], metrics[1], metrics[2], metrics[3]);

		return stance;
	}

	bool contactFromMetric(std::size_t foot_index, double metric) const
	{
		if (use_grf_hysteresis_) {
			return stance_state_[foot_index] ? metric > contact_threshold_low_ : metric > contact_threshold_high_;
		}
		return metric > contact_threshold_;
	}

	void resetStanceState()
	{
		stance_state_.fill(false);
	}

	void validateConfiguredNamesAndIndices(const std::string& label)
	{
		if (motor_joint_names_.size() != 12) {
			RCLCPP_WARN(node_->get_logger(), "%s: motor_joint_names must contain 12 entries; using Go2 defaults", label.c_str());
			motor_joint_names_ = defaultMotorJointNames();
		}
		if (motor_indices_.size() != motor_joint_names_.size()) {
			RCLCPP_WARN(node_->get_logger(), "%s: motor_indices must match motor_joint_names; using sequential defaults", label.c_str());
			motor_indices_ = state_estimator::defaultMotorIndices(motor_joint_names_.size());
		}
		if (feet_frame_names_.size() != 4) {
			RCLCPP_WARN(node_->get_logger(), "%s: feet_frame_names must contain 4 entries; using Go2 defaults", label.c_str());
			feet_frame_names_ = defaultFeetFrameNames();
		}
	}

	static std::vector<std::string> defaultMotorJointNames()
	{
		return {
			"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
			"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
			"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
			"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
		};
	}

	static std::vector<std::string> defaultFeetFrameNames()
	{
		return {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
	}

	static std::vector<std::size_t> toSizeTVector(const std::vector<int>& values)
	{
		std::vector<std::size_t> converted;
		converted.reserve(values.size());
		for (const int value : values) {
			if (value >= 0) {
				converted.push_back(static_cast<std::size_t>(value));
			}
		}
		return converted;
	}

	static Eigen::Vector3d vector3FromConfig(const std::vector<double>& values, const Eigen::Vector3d& fallback)
	{
		if (values.size() != 3) {
			return fallback;
		}
		const Eigen::Vector3d vector(values[0], values[1], values[2]);
		return (!vector.allFinite() || vector.norm() < 1e-9) ? fallback : vector.normalized();
	}

	static std::string resolvePackagePath(const std::string& input)
	{
		std::string resolved = input;
		const std::string find_token = "$(find ";
		const std::size_t token_pos = resolved.find(find_token);
		if (token_pos == std::string::npos) {
			return resolved;
		}
		const std::size_t package_start = token_pos + find_token.length();
		const std::size_t package_end = resolved.find(")", package_start);
		if (package_end == std::string::npos) {
			return resolved;
		}
		const std::string package_name = resolved.substr(package_start, package_end - package_start);
		try {
			const std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
			resolved.replace(token_pos, package_end - token_pos + 1, package_path);
		} catch (const std::exception&) {
			return input;
		}
		return resolved;
	}

	rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr go_low_state_sub_;
	rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr hg_low_state_sub_;
	rclcpp::Publisher<state_estimator_msgs::msg::LegOdometry>::SharedPtr pub_;

	state_estimator_msgs::msg::LegOdometry msg_;
	state_estimator::UnitreeLowState normalized_low_state_;
	state_estimator::AttitudeBiasXKF* attitude_{nullptr};
	state_estimator::GrfEstimator grf_estimator_;

	Eigen::Vector7d xhat_att_;
	Eigen::Vector3d f_n_, m_n_;
	Eigen::Matrix3d base_R_imu_;

	std::vector<std::string> motor_joint_names_;
	std::vector<std::size_t> motor_indices_;
	std::vector<std::string> feet_frame_names_;
	double contact_threshold_{15.0};
	double contact_threshold_low_{15.0};
	double contact_threshold_high_{15.0};
	bool use_grf_hysteresis_{false};
	std::array<bool, 4> stance_state_{{false, false, false, false}};
	bool grf_estimator_configured_{false};
	Eigen::Vector3d contact_normal_{Eigen::Vector3d::UnitZ()};
	bool use_force_norm_{false};
	bool use_absolute_normal_force_{true};

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
