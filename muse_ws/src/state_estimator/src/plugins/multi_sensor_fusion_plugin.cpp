#include "state_estimator/plugin.hpp"
#include "state_estimator/Models/multi_sensor_fusion.hpp"

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include <state_estimator_msgs/msg/attitude.hpp>
#include <state_estimator_msgs/msg/leg_odometry.hpp>
#include <state_estimator_msgs/msg/contact_detection.hpp>
#include <state_estimator_msgs/msg/sensor_fusion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <iit/commons/geometry/rotations.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  state_estimator_msgs::msg::Attitude,
  state_estimator_msgs::msg::LegOdometry,
  state_estimator_msgs::msg::ContactDetection,
  nav_msgs::msg::Odometry>;

class MultiSensorFusionPlugin : public PluginBase
{
public:
  MultiSensorFusionPlugin() = default;
  ~MultiSensorFusionPlugin() override
  {
    if (multi_sensor_fusion_ != nullptr) {
      delete multi_sensor_fusion_;
    }
  }

  std::string getName() override { return std::string("MultiSensorFusion"); }
  std::string getDescription() override { return std::string("Multi-sensor fusion using attitude, leg odometry, stance and LiDAR odometry"); }

  void initialize_() override
  {
    std::string attitude_topic = "muse/attitude";
    std::string leg_odometry_topic = "muse/leg_odometry";
    std::string stance_topic = "muse/contact_detection";
    std::string lidar_odometry_topic = "/lidar_odometry";
    std::string pub_topic = "muse/multi_sensor_fusion";
    std::string state_topic = "muse/multi_sensor_fusion_state";
    world_frame_id_ = "world";
    base_frame_id_ = "base";

    std::vector<double> x0_vec = {-0.017, 0.0, 0.383, 0.0, 0.0, 0.0};
    std::vector<double> P_vec(36, 0.0), Q_vec(36, 0.0), R_vec(81, 0.0);

    for (int i = 0; i < 6; ++i) {
      P_vec[i * 6 + i] = (i < 3) ? 1.0e-3 : 1.0e-4;
      Q_vec[i * 6 + i] = (i < 3) ? 1.0e-6 : 1.0e-5;
    }
    for (int i = 0; i < 9; ++i) {
      R_vec[i * 9 + i] = (i < 3) ? 2.0e-3 : ((i < 6) ? 5.0e-3 : 8.0e-3);
    }

    if (!config_dir_.empty()) {
      try {
        YAML::Node cfg = YAML::LoadFile(config_dir_ + "/multi_sensor_fusion.yaml")["multi_sensor_fusion_plugin"];
        if (cfg["attitude_topic"]) attitude_topic = cfg["attitude_topic"].as<std::string>();
        if (cfg["leg_odometry_topic"]) leg_odometry_topic = cfg["leg_odometry_topic"].as<std::string>();
        if (cfg["stance_topic"]) stance_topic = cfg["stance_topic"].as<std::string>();
        if (cfg["lidar_odometry_topic"]) lidar_odometry_topic = cfg["lidar_odometry_topic"].as<std::string>();
        if (cfg["pub_topic"]) pub_topic = cfg["pub_topic"].as<std::string>();
        if (cfg["state_topic"]) state_topic = cfg["state_topic"].as<std::string>();
        if (cfg["world_frame_id"]) world_frame_id_ = cfg["world_frame_id"].as<std::string>();
        if (cfg["base_frame_id"]) base_frame_id_ = cfg["base_frame_id"].as<std::string>();
        if (cfg["x0"] && cfg["x0"].as<std::vector<double>>().size() == 6) x0_vec = cfg["x0"].as<std::vector<double>>();
        if (cfg["P"] && cfg["P"].as<std::vector<double>>().size() == 36) P_vec = cfg["P"].as<std::vector<double>>();
        if (cfg["Q"] && cfg["Q"].as<std::vector<double>>().size() == 36) Q_vec = cfg["Q"].as<std::vector<double>>();
        if (cfg["R"] && cfg["R"].as<std::vector<double>>().size() == 81) R_vec = cfg["R"].as<std::vector<double>>();
      } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Could not load multi sensor fusion config: %s", e.what());
      }
    }

    Eigen::Matrix<double,6,1> x0;
    for (int i = 0; i < 6; ++i) x0(i) = x0_vec[i];

    const Eigen::Matrix<double,6,6> P0 = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(P_vec.data());
    const Eigen::Matrix<double,6,6> Q = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(Q_vec.data());
    const Eigen::Matrix<double,9,9> R = Eigen::Map<Eigen::Matrix<double,9,9,Eigen::RowMajor>>(R_vec.data());

    multi_sensor_fusion_ = new state_estimator::KFMultiSensorFusion(0.0, x0, P0, Q, R);

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(pub_topic, 250);
    state_pub_ = node_->create_publisher<state_estimator_msgs::msg::SensorFusion>(state_topic, 250);

    attitude_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>>(node_.get(), attitude_topic, rclcpp::QoS(250).get_rmw_qos_profile());
    leg_odom_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::LegOdometry>>(node_.get(), leg_odometry_topic, rclcpp::QoS(250).get_rmw_qos_profile());
    stance_sub_ = std::make_shared<message_filters::Subscriber<state_estimator_msgs::msg::ContactDetection>>(node_.get(), stance_topic, rclcpp::QoS(250).get_rmw_qos_profile());
    lidar_odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(node_.get(), lidar_odometry_topic, rclcpp::QoS(250).get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), *attitude_sub_, *leg_odom_sub_, *stance_sub_, *lidar_odom_sub_);
    sync_->registerCallback(std::bind(&MultiSensorFusionPlugin::callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3,
                                      std::placeholders::_4));

    RCLCPP_INFO(node_->get_logger(), "MultiSensorFusionPlugin initialized");
  }

  void shutdown_() override {}
  void pause_() override {}
  void resume_() override {}
  void reset_() override {}

private:
  void callback(const state_estimator_msgs::msg::Attitude::ConstSharedPtr& attitude,
                const state_estimator_msgs::msg::LegOdometry::ConstSharedPtr& leg_odom,
                const state_estimator_msgs::msg::ContactDetection::ConstSharedPtr& stance,
                const nav_msgs::msg::Odometry::ConstSharedPtr& lidar_odom)
  {
    if (begin_) {
      time_begin_ = node_->now().seconds();
      begin_ = false;
      last_lidar_stamp_ = lidar_odom->header.stamp;
      last_lidar_vel_ << lidar_odom->twist.twist.linear.x,
                         lidar_odom->twist.twist.linear.y,
                         lidar_odom->twist.twist.linear.z;
    }

    const double t = node_->now().seconds() - time_begin_;

    Eigen::Quaterniond quat_est(attitude->quaternion[0], attitude->quaternion[1], attitude->quaternion[2], attitude->quaternion[3]);
    if (!std::isfinite(quat_est.norm()) || quat_est.norm() < 1e-9) {
      quat_est = Eigen::Quaterniond::Identity();
    } else {
      quat_est.normalize();
    }

    Eigen::Vector3d rpy = iit::commons::quatToRPY(quat_est);
    rpy(0) = 0.0;
    rpy(1) = 0.0;
    const Eigen::Matrix3d w_R_b = iit::commons::rpyToRot(rpy).transpose();

    Eigen::Vector3d v_b(leg_odom->base_velocity[0], leg_odom->base_velocity[1], leg_odom->base_velocity[2]);
    const Eigen::Vector3d w_v_b = w_R_b * v_b;

    const Eigen::Vector3d w_p_lidar(lidar_odom->pose.pose.position.x,
                                    lidar_odom->pose.pose.position.y,
                                    lidar_odom->pose.pose.position.z);
    const Eigen::Vector3d w_v_lidar(lidar_odom->twist.twist.linear.x,
                                    lidar_odom->twist.twist.linear.y,
                                    lidar_odom->twist.twist.linear.z);

    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    const rclcpp::Time cur_lidar_stamp(lidar_odom->header.stamp);
    if (last_lidar_stamp_.nanosec != 0 || last_lidar_stamp_.sec != 0) {
      const double dt_lidar = (cur_lidar_stamp - rclcpp::Time(last_lidar_stamp_)).seconds();
      if (dt_lidar > 1e-4) {
        u = (w_v_lidar - last_lidar_vel_) / dt_lidar;
      }
    }
    last_lidar_vel_ = w_v_lidar;
    last_lidar_stamp_ = lidar_odom->header.stamp;

    const int stance_count = static_cast<int>(stance->stance_lf) +
                             static_cast<int>(stance->stance_rf) +
                             static_cast<int>(stance->stance_lh) +
                             static_cast<int>(stance->stance_rh);

    multi_sensor_fusion_->predict(t, u);
    multi_sensor_fusion_->setMeasurementCovariances(stance_count);

    Eigen::Matrix<double,9,1> z;
    z.segment<3>(0) = w_v_b;
    z.segment<3>(3) = w_p_lidar;
    z.segment<3>(6) = w_v_lidar;
    multi_sensor_fusion_->update(t, z);

    const Eigen::Matrix<double,6,1> xhat = multi_sensor_fusion_->getX();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_->now();
    odom_msg.header.frame_id = world_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = xhat(0);
    odom_msg.pose.pose.position.y = xhat(1);
    odom_msg.pose.pose.position.z = xhat(2);
    odom_msg.pose.pose.orientation.w = quat_est.w();
    odom_msg.pose.pose.orientation.x = quat_est.x();
    odom_msg.pose.pose.orientation.y = quat_est.y();
    odom_msg.pose.pose.orientation.z = quat_est.z();
    odom_msg.twist.twist.linear.x = xhat(3);
    odom_msg.twist.twist.linear.y = xhat(4);
    odom_msg.twist.twist.linear.z = xhat(5);
    odom_msg.twist.twist.angular.x = attitude->angular_velocity[0];
    odom_msg.twist.twist.angular.y = attitude->angular_velocity[1];
    odom_msg.twist.twist.angular.z = attitude->angular_velocity[2];
    odom_pub_->publish(odom_msg);

    state_estimator_msgs::msg::SensorFusion sf_msg;
    sf_msg.header = odom_msg.header;
    sf_msg.position[0] = xhat(0);
    sf_msg.position[1] = xhat(1);
    sf_msg.position[2] = xhat(2);
    sf_msg.linear_velocity[0] = xhat(3);
    sf_msg.linear_velocity[1] = xhat(4);
    sf_msg.linear_velocity[2] = xhat(5);
    state_pub_->publish(sf_msg);
  }

  state_estimator::KFMultiSensorFusion* multi_sensor_fusion_{nullptr};

  std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::Attitude>> attitude_sub_;
  std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::LegOdometry>> leg_odom_sub_;
  std::shared_ptr<message_filters::Subscriber<state_estimator_msgs::msg::ContactDetection>> stance_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> lidar_odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<state_estimator_msgs::msg::SensorFusion>::SharedPtr state_pub_;

  bool begin_{true};
  double time_begin_{0.0};

  std::string world_frame_id_;
  std::string base_frame_id_;

  builtin_interfaces::msg::Time last_lidar_stamp_{};
  Eigen::Vector3d last_lidar_vel_{Eigen::Vector3d::Zero()};
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::MultiSensorFusionPlugin, state_estimator_plugins::PluginBase)
