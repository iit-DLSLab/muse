#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <iit/commons/geometry/rotations.h>

#include "state_estimator/plugin.hpp"
#include "state_estimator/Models/attitude_bias_XKF.hpp"
#include "state_estimator/lib.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "state_estimator_msgs/msg/leg_odometry.hpp"
#include "unitree_go/msg/low_state.hpp"

#include <yaml-cpp/yaml.h>
#include <unordered_map>


namespace state_estimator_plugins
{

//  This class calculates the base velocity as estimated from the leg kinematics.
//  It subscribes only to LowState and manages its own internal attitude estimator.
class LegOdometryPlugin : public PluginBase
{
public:
LegOdometryPlugin() : attitude_(nullptr) {}
~LegOdometryPlugin() { if (attitude_ != nullptr) delete attitude_; }

std::string getName() override { return std::string("LegOdometry"); }
std::string getDescription() override { return std::string("Leg Odometry Plugin"); }

void initialize_() override 
{
t0_ = 0.0;
std::string urdf_path_param = "";
std::string low_state_topic    = "/lowstate";
std::string pub_topic          = "/state_estimator/leg_odometry";
std::vector<double> base_R_imu_vec(9, 0.0);
double ki_param = 0.02, kp_param = 10.0;
std::vector<double> P_vec, Q_vec, R_vec;

// Default Go1 motor order: FL(hip,thigh,calf), FR, RL, RR
motor_joint_names_ = {
"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
};
feet_frame_names_ = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};

if (!config_dir_.empty()) {
try {
YAML::Node lo_cfg = YAML::LoadFile(config_dir_ + "/leg_odometry.yaml")["leg_odometry_plugin"];
if (lo_cfg["urdf_path"])          urdf_path_param   = lo_cfg["urdf_path"].as<std::string>();
if (lo_cfg["low_state_topic"])    low_state_topic   = lo_cfg["low_state_topic"].as<std::string>();
if (lo_cfg["pub_topic"])          pub_topic         = lo_cfg["pub_topic"].as<std::string>();
if (lo_cfg["contact_force_threshold"]) contact_threshold_ = lo_cfg["contact_force_threshold"].as<double>();
if (lo_cfg["motor_joint_names"])  motor_joint_names_ = lo_cfg["motor_joint_names"].as<std::vector<std::string>>();
if (lo_cfg["feet_frame_names"])   feet_frame_names_ = lo_cfg["feet_frame_names"].as<std::vector<std::string>>();

YAML::Node att_cfg = YAML::LoadFile(config_dir_ + "/attitude_plugin.yaml")["attitude_estimation_plugin"];
if (att_cfg["base_R_imu"])    base_R_imu_vec = att_cfg["base_R_imu"].as<std::vector<double>>();
if (att_cfg["ki"])            ki_param         = att_cfg["ki"].as<double>();
if (att_cfg["kp"])            kp_param         = att_cfg["kp"].as<double>();
if (att_cfg["P"])             P_vec            = att_cfg["P"].as<std::vector<double>>();
if (att_cfg["Q"])             Q_vec            = att_cfg["Q"].as<std::vector<double>>();
if (att_cfg["R"])             R_vec            = att_cfg["R"].as<std::vector<double>>();
if (att_cfg["gravity_vector"])gravity_vec_     = att_cfg["gravity_vector"].as<std::vector<double>>();
if (att_cfg["north_vector"])  north_vec_       = att_cfg["north_vector"].as<std::vector<double>>();
} catch (const std::exception& e) {
RCLCPP_WARN(node_->get_logger(), "Could not load leg odometry config: %s", e.what());
}
}

// Resolve $(find <pkg>) token to ament share directory
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
RCLCPP_INFO(node_->get_logger(), "URDF loaded into Pinocchio model.");
} catch (const std::exception& e) {
RCLCPP_ERROR(node_->get_logger(), "Failed to load URDF: %s", e.what());
}

// Setup base rotation and gravity/north vectors
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
xhat_att_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
xhat_att_.head(4) = xhat_att_.head(4) / xhat_att_.head(4).norm();

if (P_vec.size() == 36 && Q_vec.size() == 36 && R_vec.size() == 36) {
Eigen::Matrix6d P0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(P_vec.data());
Eigen::Matrix6d Q  = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(Q_vec.data());
Eigen::Matrix6d R  = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(R_vec.data());
attitude_ = new state_estimator::AttitudeBiasXKF(t0_, xhat_att_, P0, Q, R, f_n_, m_n_, ki_param, kp_param);
} else {
RCLCPP_WARN(node_->get_logger(), "Missing/invalid P/Q/R — using scaled identity matrices.");
Eigen::Matrix6d P0 = 1e-6 * Eigen::Matrix6d::Identity();
Eigen::Matrix6d Q  = 1e-6 * Eigen::Matrix6d::Identity();
Eigen::Matrix6d R  = 1e-2 * Eigen::Matrix6d::Identity();
attitude_ = new state_estimator::AttitudeBiasXKF(t0_, xhat_att_, P0, Q, R, f_n_, m_n_, ki_param, kp_param);
}

low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
low_state_topic, 250,
std::bind(&LegOdometryPlugin::callback, this, std::placeholders::_1));

pub_ = node_->create_publisher<state_estimator_msgs::msg::LegOdometry>(pub_topic, 250);
RCLCPP_INFO(node_->get_logger(), "LegOdometryPlugin initialized on topic '%s'", low_state_topic.c_str());
}

void shutdown_() override { }
void pause_() override { }
void resume_() override { }
void reset_() override { }

void callback(const unitree_go::msg::LowState::SharedPtr low_state)
{
if (begin_) {
time_begin_ = node_->now().seconds();
begin_ = false;
}

time_ = node_->now().seconds() - time_begin_;

// Extract IMU data from LowState
Eigen::Vector3d omega(
low_state->imu_state.gyroscope[0],
low_state->imu_state.gyroscope[1],
low_state->imu_state.gyroscope[2]);
Eigen::Vector3d acc(
low_state->imu_state.accelerometer[0],
low_state->imu_state.accelerometer[1],
low_state->imu_state.accelerometer[2]);

// Update internal attitude estimator with IMU from LowState
Eigen::Vector3d f_b = base_R_imu_ * acc;
Eigen::Vector3d omega_b = base_R_imu_ * omega;
Eigen::Quaterniond quat_est;
quat_est.w() = xhat_att_(0);
quat_est.vec() << xhat_att_(1), xhat_att_(2), xhat_att_(3);
Eigen::Vector3d m_b = iit::commons::quatToRotMat(quat_est) * m_n_;
Eigen::Matrix<double,6,1> z;
z << f_b, m_b;
attitude_->update(time_, omega_b, z);
xhat_att_ = attitude_->getX();

// Extract updated quaternion and rotation matrix for kinematics
quat_est.w() = xhat_att_(0);
quat_est.vec() << xhat_att_(1), xhat_att_(2), xhat_att_(3);
Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();
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

std::vector<Eigen::Vector3d> foot_vels;
for (size_t i = 0; i < feet_frame_names_.size(); ++i) {
const auto& foot_name = feet_frame_names_[i];
std::size_t frame_id = model_.getFrameId(foot_name);
if (frame_id >= model_.nframes) {
RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                     "LegOdometry: foot frame '%s' not found in model", foot_name.c_str());
foot_vels.push_back(Eigen::Vector3d::Zero());
continue;
}

// With a fixed-base model, WORLD quantities are expressed in the base frame.
pinocchio::Motion foot_vel_base = pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::WORLD);
Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();

// For a stance foot: 0 = v_b + ω×r + v_foot/base  ⇒  v_b = -(v_foot/base + ω×r)
Eigen::Vector3d rel_vel = -(foot_vel_base.linear() + base_omega.cross(foot_pos_base));
foot_vels.push_back(rel_vel);
}

Eigen::Vector3d lin_leg_lf = foot_vels[0];
Eigen::Vector3d lin_leg_rf = foot_vels[1];
Eigen::Vector3d lin_leg_lh = foot_vels[2];
Eigen::Vector3d lin_leg_rh = foot_vels[3];

// Average foot velocities for stance feet
double sum_stance = stance_lf + stance_rf + stance_lh + stance_rh;
Eigen::Vector3d base_velocity = (stance_lf*lin_leg_lf + stance_rf*lin_leg_rf + 
  stance_lh*lin_leg_lh + stance_rh*lin_leg_rh) / (sum_stance + 1e-5);
base_velocity = w_R_b * base_velocity;

// Publish results
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

private:
std::shared_ptr<rclcpp::Subscription<unitree_go::msg::LowState>> low_state_sub_;
std::shared_ptr<rclcpp::Publisher<state_estimator_msgs::msg::LegOdometry>> pub_;

state_estimator_msgs::msg::LegOdometry msg_;
state_estimator::AttitudeBiasXKF* attitude_{nullptr};
Eigen::Vector7d xhat_att_;
Eigen::Vector3d f_n_, m_n_;

std::vector<std::string> motor_joint_names_;
double contact_threshold_{30.0};
double t0_{0.0};
double time_{0.0};
bool begin_{true};
double time_begin_{0.0};
std::vector<double> gravity_vec_, north_vec_;

pinocchio::Model model_;
pinocchio::Data data_;
std::vector<std::string> feet_frame_names_;
bool model_loaded_{false};

Eigen::Vector3d omega;
bool stance_lf{false};
bool stance_rf{false};
bool stance_lh{false};
bool stance_rh{false};
Eigen::Matrix3d base_R_imu_;
};

} // end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::LegOdometryPlugin, state_estimator_plugins::PluginBase)
