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
#include "state_estimator_msgs/msg/base_height.hpp"
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
#include <fstream>

namespace state_estimator_plugins
{

using ImuMsg = sensor_msgs::msg::Imu;
using JointStateMsg = sensor_msgs::msg::JointState;
using ContactMsg = state_estimator_msgs::msg::ContactDetection;
using AttitudeMsg = state_estimator_msgs::msg::Attitude;

using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, JointStateMsg, ContactMsg, AttitudeMsg>;
using ExactTimePolicy = message_filters::sync_policies::ExactTime<ImuMsg, JointStateMsg, ContactMsg, AttitudeMsg>;

#define MySyncPolicy ApproximateTimePolicy

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

        // Initialize member variables
        omega_.setZero();
        base_omega_.setZero();
        stance_lf_ = false;
        stance_rf_ = false;
        stance_lh_ = false;
        stance_rh_ = false;
        base_R_imu_.setIdentity();
        model_loaded_ = false;

        // Load URDF
        if (!loadURDF()) {
            throw std::runtime_error("Failed to load URDF model");
        }

        // Configure rotation matrix
        configureRotationMatrix();

        // Configure feet frame names
        configureFeetFrames();

        // Configure joint order
        configureJointOrder();

        // Setup subscribers and publishers
        setupTopics();

        RCLCPP_INFO(node->get_logger(), "LegOdometryPlugin initialized successfully");
    }

    void shutdown_() override {}
    void pause_() override {}
    void resume_() override {}
    void reset_() override {}

private:
    // Core components
    std::shared_ptr<message_filters::Subscriber<ImuMsg>> imu_sub_;
    std::shared_ptr<message_filters::Subscriber<JointStateMsg>> joint_state_sub_;
    std::shared_ptr<message_filters::Subscriber<ContactMsg>> contact_sub_;
    std::shared_ptr<message_filters::Subscriber<AttitudeMsg>> attitude_sub_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    rclcpp::Publisher<state_estimator_msgs::msg::LegOdometry>::SharedPtr pub_;
    rclcpp::Publisher<state_estimator_msgs::msg::BaseHeight>::SharedPtr base_height_pub_;

    state_estimator_msgs::msg::LegOdometry msg_;
    state_estimator_msgs::msg::BaseHeight base_height_msg_;

    // Pinocchio model
    pinocchio::Model model_;
    pinocchio::Data data_;
    bool model_loaded_;

    // Configuration
    std::vector<std::string> feet_frame_names_;
    std::vector<std::string> expected_joint_order_;
    bool use_explicit_joint_order_;
    Eigen::Matrix3d base_R_imu_;

    // State variables
    Eigen::Vector3d omega_;
    Eigen::Vector3d base_omega_;
    bool stance_lf_, stance_rf_, stance_lh_, stance_rh_;

    // Frame IDs for efficiency
    std::vector<pinocchio::FrameIndex> foot_frame_ids_;

    bool loadURDF() {
        auto node = this->node_;
        
        // Get URDF path
        std::string urdf_path = node->declare_parameter<std::string>(
            "leg_odometry_plugin.urdf_path", "");
        
        // Check legacy parameter
        const std::string legacy_urdf = node->declare_parameter<std::string>(
            "leg_odometry_plugin/urdf_path", "");
        if (!legacy_urdf.empty() && urdf_path.empty()) {
            RCLCPP_WARN(node->get_logger(),
                "Using legacy param 'leg_odometry_plugin/urdf_path'; prefer 'leg_odometry_plugin.urdf_path'");
            urdf_path = legacy_urdf;
        }

        if (urdf_path.empty()) {
            RCLCPP_ERROR(node->get_logger(), "URDF path parameter not set");
            return false;
        }

        // Resolve package path
        std::string resolved_path = resolvePackagePath(urdf_path);
        
        // Check if file exists
        std::ifstream file_check(resolved_path);
        if (!file_check.good()) {
            RCLCPP_ERROR(node->get_logger(), "URDF file does not exist: %s", resolved_path.c_str());
            return false;
        }

        RCLCPP_INFO(node->get_logger(), "Loading URDF from: %s", resolved_path.c_str());

        try {
            pinocchio::urdf::buildModel(resolved_path, model_);
            data_ = pinocchio::Data(model_);
            model_loaded_ = true;
            
            RCLCPP_INFO(node->get_logger(), "URDF loaded successfully");
            RCLCPP_INFO(node->get_logger(), "Model DOF: nq=%d, nv=%d, njoints=%zu", 
                       model_.nq, model_.nv, model_.njoints);
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Failed to load URDF: %s", e.what());
            return false;
        }
    }

    std::string resolvePackagePath(const std::string& path) {
        std::string resolved_path = path;
        const std::string find_token = "$(find ";
        
        if (resolved_path.find(find_token) != std::string::npos) {
            const size_t start = resolved_path.find(find_token) + find_token.length();
            const size_t end = resolved_path.find(')', start);
            
            if (end != std::string::npos) {
                const std::string package_name = resolved_path.substr(start, end - start);
                try {
                    const std::string pkg_share = ament_index_cpp::get_package_share_directory(package_name);
                    resolved_path.replace(resolved_path.find(find_token), 
                                        end - resolved_path.find(find_token) + 1, pkg_share);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->node_->get_logger(), 
                               "Failed to resolve package '%s': %s", package_name.c_str(), e.what());
                }
            }
        }
        
        return resolved_path;
    }

    void configureRotationMatrix() {
        auto node = this->node_;
        
        std::vector<double> base_R_imu_vec = node->declare_parameter<std::vector<double>>( 
            "leg_odometry_plugin.base_R_imu", std::vector<double>());
            
        if (base_R_imu_vec.size() == 9) {
            base_R_imu_ = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(base_R_imu_vec.data());
            RCLCPP_INFO(node->get_logger(), "Using custom base_R_imu rotation matrix");
        } else {
            if (!base_R_imu_vec.empty()) {
                RCLCPP_WARN(node->get_logger(), 
                           "base_R_imu parameter size (%zu) != 9; using identity", base_R_imu_vec.size());
            }
            base_R_imu_.setIdentity();
        }
    }

    void configureFeetFrames() {
        auto node = this->node_;
        
        // Default frame names
        feet_frame_names_ = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
        
        auto frames_param = node->declare_parameter<std::vector<std::string>>(
            "leg_odometry_plugin.feet_frame_names", std::vector<std::string>{}
        );
        auto frames_legacy = node->declare_parameter<std::vector<std::string>>(
            "leg_odometry_plugin/feet_frame_names", std::vector<std::string>{}
        );
        
        if (!frames_legacy.empty() && frames_param.empty()) {
            RCLCPP_WARN(node->get_logger(),
                "Using legacy param 'leg_odometry_plugin/feet_frame_names'");
            frames_param = frames_legacy;
        }
        
        if (!frames_param.empty()) {
            if (frames_param.size() != 4) {
                RCLCPP_WARN(node->get_logger(),
                    "feet_frame_names must contain exactly 4 names, got %zu. Using defaults.",
                    frames_param.size());
            } else {
                feet_frame_names_ = frames_param;
            }
        }

        // Validate and cache frame IDs
        foot_frame_ids_.clear();
        for (const auto& frame_name : feet_frame_names_) {
            if (!model_.existFrame(frame_name)) {
                throw std::runtime_error("Frame '" + frame_name + "' does not exist in URDF model");
            }
            foot_frame_ids_.push_back(model_.getFrameId(frame_name));
        }

        RCLCPP_INFO(node->get_logger(), "Using feet frames: [%s, %s, %s, %s]",
                   feet_frame_names_[0].c_str(), feet_frame_names_[1].c_str(),
                   feet_frame_names_[2].c_str(), feet_frame_names_[3].c_str());
    }

    void configureJointOrder() {
        auto node = this->node_;
        
        auto joint_order_param = node->declare_parameter<std::vector<std::string>>(
            "leg_odometry_plugin.joint_order", std::vector<std::string>{}
        );
        auto joint_order_legacy = node->declare_parameter<std::vector<std::string>>(
            "leg_odometry_plugin/joint_order", std::vector<std::string>{}
        );
        
        if (!joint_order_legacy.empty() && joint_order_param.empty()) {
            RCLCPP_WARN(node->get_logger(),
                "Using legacy param 'leg_odometry_plugin/joint_order'");
            joint_order_param = joint_order_legacy;
        }

        use_explicit_joint_order_ = false;
        
        if (!joint_order_param.empty()) {
            if (validateJointOrder(joint_order_param)) {
                expected_joint_order_ = joint_order_param;
                use_explicit_joint_order_ = true;
                RCLCPP_INFO(node->get_logger(), "Using explicit joint order with %zu joints", 
                           expected_joint_order_.size());
            }
        } else {
            RCLCPP_INFO(node->get_logger(), "Using URDF model joint order");
        }
    }

    bool validateJointOrder(const std::vector<std::string>& joint_order) {
        auto node = this->node_;
        
        if (joint_order.size() != static_cast<size_t>(model_.nq)) {
            RCLCPP_WARN(node->get_logger(),
                "Joint order size (%zu) != model DOF (%d)", joint_order.size(), model_.nq);
            return false;
        }

        // Check all joints exist in model
        for (const auto& joint_name : joint_order) {
            bool found = false;
            for (pinocchio::JointIndex i = 1; i < model_.njoints; ++i) {
                if (model_.names[i] == joint_name) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_WARN(node->get_logger(), "Joint '%s' not found in model", joint_name.c_str());
                return false;
            }
        }

        return true;
    }

    void setupTopics() {
        auto node = this->node_;
        
        // Get topic names
        std::string imu_topic = node->declare_parameter<std::string>(
            "leg_odometry_plugin.imu_topic", "/imu");
        std::string joint_states_topic = node->declare_parameter<std::string>(
            "leg_odometry_plugin.joint_states_topic", "/state_estimator/joint_states");
        std::string contact_topic = node->declare_parameter<std::string>(
            "leg_odometry_plugin.contact_topic", "/state_estimator/contact_detection");
        std::string attitude_topic = node->declare_parameter<std::string>(
            "leg_odometry_plugin.attitude_topic", "/state_estimator/attitude");

        // Setup subscribers
        auto sensor_qos = rclcpp::SensorDataQoS();
        
        imu_sub_ = std::make_shared<message_filters::Subscriber<ImuMsg>>(node, imu_topic, sensor_qos);
        joint_state_sub_ = std::make_shared<message_filters::Subscriber<JointStateMsg>>(node, joint_states_topic, sensor_qos);
        contact_sub_ = std::make_shared<message_filters::Subscriber<ContactMsg>>(node, contact_topic, sensor_qos);
        attitude_sub_ = std::make_shared<message_filters::Subscriber<AttitudeMsg>>(node, attitude_topic, sensor_qos);

        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(100), *imu_sub_, *joint_state_sub_, *contact_sub_, *attitude_sub_);
        sync_->registerCallback(std::bind(&LegOdometryPlugin::callback, this,
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4));

        // Setup publishers
        std::string pub_topic = node->declare_parameter<std::string>(
            "leg_odometry_plugin.pub_topic", "/state_estimator/leg_odometry");
        std::string base_height_topic = node->declare_parameter<std::string>(
            "leg_odometry_plugin.base_height_topic", "/state_estimator/base_height");

        pub_ = node->create_publisher<state_estimator_msgs::msg::LegOdometry>(pub_topic, rclcpp::QoS(10));
        base_height_pub_ = node->create_publisher<state_estimator_msgs::msg::BaseHeight>(base_height_topic, rclcpp::QoS(10));
    }

    void callback(const ImuMsg::ConstSharedPtr imu,
                  const JointStateMsg::ConstSharedPtr js,
                  const ContactMsg::ConstSharedPtr contact,
                  const AttitudeMsg::ConstSharedPtr attitude) {
        
        if (!model_loaded_) {
            RCLCPP_ERROR_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 5000,
                                 "Model not loaded, skipping callback");
            return;
        }

        // Validate input sizes
        if (!validateInputSizes(js)) {
            return;
        }

        // Fill joint state vectors
        Eigen::VectorXd q(model_.nq), v(model_.nv);
        if (!fillJointVectors(js, q, v)) {
            return;
        }

        // Extract IMU and contact data
        extractSensorData(imu, contact);

        // Compute kinematics
        std::vector<Eigen::Vector3d> foot_velocities;
        if (!computeFootVelocities(q, v, foot_velocities)) {
            return;
        }

        // Compute base velocity from leg odometry
        Eigen::Vector3d base_velocity = computeBaseVelocity(foot_velocities, attitude);

        // Compute base height
        double base_height;
        std::vector<double> foot_heights;
        int num_contacts;
        computeBaseHeight(foot_heights, base_height, num_contacts);

        // Publish results
        publishResults(base_velocity, foot_velocities, base_height, foot_heights, num_contacts);
    }

    bool validateInputSizes(const JointStateMsg::ConstSharedPtr& js) {
        if (js->position.size() != static_cast<size_t>(model_.nq) || 
            js->velocity.size() != static_cast<size_t>(model_.nv)) {
            RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 5000,
                                 "Joint state size mismatch (pos=%zu vs nq=%d, vel=%zu vs nv=%d)",
                                 js->position.size(), model_.nq, js->velocity.size(), model_.nv);
            return false;
        }
        return true;
    }

    bool fillJointVectors(const JointStateMsg::ConstSharedPtr& js, 
                         Eigen::VectorXd& q, Eigen::VectorXd& v) {
        
        // Build joint maps for lookup
        std::unordered_map<std::string, double> joint_pos_map, joint_vel_map;
        for (size_t i = 0; i < js->name.size(); ++i) {
            joint_pos_map[js->name[i]] = js->position[i];
            if (i < js->velocity.size()) {
                joint_vel_map[js->name[i]] = js->velocity[i];
            }
        }

        if (use_explicit_joint_order_) {
            return fillJointVectorsExplicit(joint_pos_map, joint_vel_map, q, v);
        } else {
            return fillJointVectorsModel(joint_pos_map, joint_vel_map, q, v);
        }
    }

    bool fillJointVectorsExplicit(const std::unordered_map<std::string, double>& pos_map,
                                 const std::unordered_map<std::string, double>& vel_map,
                                 Eigen::VectorXd& q, Eigen::VectorXd& v) {
        
        for (size_t i = 0; i < expected_joint_order_.size(); ++i) {
            const std::string& joint_name = expected_joint_order_[i];
            
            auto pos_it = pos_map.find(joint_name);
            auto vel_it = vel_map.find(joint_name);
            
            if (pos_it == pos_map.end()) {
                RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 5000,
                    "Joint '%s' not found in message", joint_name.c_str());
                q[i] = 0.0;
            } else {
                q[i] = pos_it->second;
            }

            if (vel_it == vel_map.end()) {
                v[i] = 0.0;
            } else {
                v[i] = vel_it->second;
            }
        }
        return true;
    }

    bool fillJointVectorsModel(const std::unordered_map<std::string, double>& pos_map,
                              const std::unordered_map<std::string, double>& vel_map,
                              Eigen::VectorXd& q, Eigen::VectorXd& v) {
        
        for (pinocchio::JointIndex i = 1; i < model_.njoints; ++i) {
            const std::string& joint_name = model_.names[i];
            
            auto pos_it = pos_map.find(joint_name);
            auto vel_it = vel_map.find(joint_name);
            
            q[i - 1] = (pos_it != pos_map.end()) ? pos_it->second : 0.0;
            v[i - 1] = (vel_it != vel_map.end()) ? vel_it->second : 0.0;
        }
        return true;
    }

    void extractSensorData(const ImuMsg::ConstSharedPtr& imu, 
                          const ContactMsg::ConstSharedPtr& contact) {
        omega_ << imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z;
        stance_lf_ = contact->stance_lf;
        stance_rf_ = contact->stance_rf;
        stance_lh_ = contact->stance_lh;
        stance_rh_ = contact->stance_rh;
    }

    bool computeFootVelocities(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                              std::vector<Eigen::Vector3d>& foot_velocities) {
        try {
            // Compute forward kinematics
            pinocchio::forwardKinematics(model_, data_, q, v);
            pinocchio::updateFramePlacements(model_, data_);

            foot_velocities.clear();
            foot_velocities.reserve(4);

            // Compute angular velocity in base frame
            Eigen::Vector3d omega_rotated = base_R_imu_ * omega_;

            for (size_t i = 0; i < foot_frame_ids_.size(); ++i) {
                pinocchio::FrameIndex frame_id = foot_frame_ids_[i];
                
                // Get foot velocity in world frame
                pinocchio::Motion foot_vel = pinocchio::getFrameVelocity(
                    model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

                // Get foot position relative to base
                Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();

                // Compute velocity contribution from base angular motion: ω × r
                Eigen::Vector3d omega_cross_r = omega_rotated.cross(foot_pos_base);

                // Compute foot velocity relative to base (negative because we want base motion)
                Eigen::Vector3d rel_vel = -(foot_vel.linear() - omega_cross_r);
                
                foot_velocities.push_back(rel_vel);
            }
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 5000,
                                 "Failed to compute foot velocities: %s", e.what());
            return false;
        }
    }

    Eigen::Vector3d computeBaseVelocity(const std::vector<Eigen::Vector3d>& foot_velocities,
                                       const AttitudeMsg::ConstSharedPtr& attitude) {
        
        // Get stance indicators
        std::vector<bool> stances = {stance_lf_, stance_rf_, stance_lh_, stance_rh_};
        
        // Compute weighted average of foot velocities
        Eigen::Vector3d base_velocity = Eigen::Vector3d::Zero();
        double total_weight = 0.0;
        
        for (size_t i = 0; i < foot_velocities.size() && i < stances.size(); ++i) {
            if (stances[i]) {
                base_velocity += foot_velocities[i];
                total_weight += 1.0;
            }
        }
        
        if (total_weight > 0.0) {
            base_velocity /= total_weight;
        }

        // Transform to world frame using attitude
        Eigen::Quaterniond quat_est;
        quat_est.w() = attitude->quaternion[0];
        quat_est.vec() << attitude->quaternion[1], attitude->quaternion[2], attitude->quaternion[3];
        
        Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est);
        
        return w_R_b * base_velocity;
    }

    void computeBaseHeight(std::vector<double>& foot_heights, double& estimated_height, int& num_contacts) {
        foot_heights.clear();
        foot_heights.reserve(4);
        
        std::vector<bool> stances = {stance_lf_, stance_rf_, stance_lh_, stance_rh_};
        
        double total_weighted_height = 0.0;
        num_contacts = 0;
        
        for (size_t i = 0; i < foot_frame_ids_.size(); ++i) {
            pinocchio::FrameIndex frame_id = foot_frame_ids_[i];
            
            // Get foot position in base frame
            Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();
            
            // Height is negative z-position (assuming z-up convention)
            double foot_height = -foot_pos_base.z();
            foot_heights.push_back(foot_height);
            
            // Add to weighted average if foot is in contact
            if (i < stances.size() && stances[i]) {
                total_weighted_height += foot_height;
                num_contacts++;
            }
        }
        
        estimated_height = (num_contacts > 0) ? (total_weighted_height / num_contacts) : 0.0;
    }

    void publishResults(const Eigen::Vector3d& base_velocity,
                       const std::vector<Eigen::Vector3d>& foot_velocities,
                       double base_height,
                       const std::vector<double>& foot_heights,
                       int num_contacts) {
        
        auto now = this->node_->get_clock()->now();
        
        // Publish leg odometry
        msg_.header.stamp = now;
        
        for (int j = 0; j < 3; ++j) {
            if (foot_velocities.size() >= 4) {
                msg_.lin_vel_lf[j] = foot_velocities[0][j];
                msg_.lin_vel_rf[j] = foot_velocities[1][j];
                msg_.lin_vel_lh[j] = foot_velocities[2][j];
                msg_.lin_vel_rh[j] = foot_velocities[3][j];
            }
            msg_.base_velocity[j] = base_velocity[j];
        }
        
        pub_->publish(msg_);

        // Publish base height
        base_height_msg_.header.stamp = now;
        base_height_msg_.height = base_height;
        
        if (foot_heights.size() >= 4) {
            base_height_msg_.height_lf = foot_heights[0];
            base_height_msg_.height_rf = foot_heights[1];
            base_height_msg_.height_lh = foot_heights[2];
            base_height_msg_.height_rh = foot_heights[3];
        }
        
        base_height_msg_.stance_lf = stance_lf_;
        base_height_msg_.stance_rf = stance_rf_;
        base_height_msg_.stance_lh = stance_lh_;
        base_height_msg_.stance_rh = stance_rh_;
        base_height_msg_.num_feet_in_contact = num_contacts;
        
        base_height_pub_->publish(base_height_msg_);
    }
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::LegOdometryPlugin, state_estimator_plugins::PluginBase)
