#ifndef STATE_ESTIMATOR_NODE_HPP
#define STATE_ESTIMATOR_NODE_HPP

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <inttypes.h>
#include <stdint.h>
#include <map>
#include <vector>
#include <string>
#include <iomanip>
#include <fnmatch.h>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pluginlib/class_loader.hpp>

#include "state_estimator/plugin.hpp"
#include "state_estimator/lib.hpp"
#include "state_estimator/Robot.hpp"
#include "state_estimator/Services.hpp"

namespace state_estimator {

class state_estimator_node
  : public rclcpp::Node
    // public rclcpp::Node::enable_shared_from_this<state_estimator_node>
{
public:
    explicit state_estimator_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~state_estimator_node();

    // Must be called AFTER std::make_shared<state_estimator_node>()
    // This loads plugins (avoids shared_from_this in ctor)
    void init_plugins();

    void shutdown();
    void time_out_thread();
    void stop_all_threads();

protected:
    std::shared_ptr<Robot> robot;
    pluginlib::ClassLoader<state_estimator_plugins::PluginBase> plugin_loader;
    std::vector<std::shared_ptr<state_estimator_plugins::PluginBase>> loaded_plugins;
    
    // Service servers
    rclcpp::Service<state_estimator_msgs::srv::GetActiveEstimators>::SharedPtr get_active_estimators_srv_;
    rclcpp::Service<state_estimator_msgs::srv::GetBlacklist>::SharedPtr get_blacklist_srv_;
    rclcpp::Service<state_estimator_msgs::srv::GetWhitelist>::SharedPtr get_whitelist_srv_;
    rclcpp::Service<state_estimator_msgs::srv::ListAllEstimators>::SharedPtr list_all_estimators_srv_;
    rclcpp::Service<state_estimator_msgs::srv::PauseEstimator>::SharedPtr pause_estimator_srv_;
    rclcpp::Service<state_estimator_msgs::srv::ResetEstimator>::SharedPtr reset_estimator_srv_;
    rclcpp::Service<state_estimator_msgs::srv::RestartEstimator>::SharedPtr restart_estimator_srv_;
    rclcpp::Service<state_estimator_msgs::srv::ResumeEstimator>::SharedPtr resume_estimator_srv_;
    rclcpp::Service<state_estimator_msgs::srv::StartEstimator>::SharedPtr start_estimator_srv_;
    rclcpp::Service<state_estimator_msgs::srv::StopEstimator>::SharedPtr stop_estimator_srv_;
    rclcpp::Service<state_estimator_msgs::srv::GetEstimatorDescription>::SharedPtr get_estimator_description_srv_;

    // Threading
    std::atomic<bool> shutdown_requested_{false};
    std::thread timeout_thread_;
    std::mutex plugins_mutex_;

    // Parameter & plugin helpers
    void readParamsFromROSParameterServer();
    bool add_plugin(std::string &name);
    bool add_plugin(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist);
    bool is_blacklisted(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist);
    bool pattern_match(std::string &pattern, std::string &pl_name);
    inline bool getWhitelist(std::vector<std::string> &whitelist);
    inline bool getBlacklist(std::vector<std::string> &blacklist);

    // Service callbacks
    void getActiveEstimators(
        const std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Response> response);
        
    void getBlacklistService(
        const std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Response> response);
        
    void getEstimatorDescription(
        const std::shared_ptr<state_estimator_msgs::srv::GetEstimatorDescription::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::GetEstimatorDescription::Response> response);
        
    void getWhitelistService(
        const std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Response> response);
        
    void listAllEstimators(
        const std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Response> response);
        
    void pauseEstimator(
        const std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Response> response);
        
    void resetEstimator(
        const std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Response> response);
        
    void restartEstimator(
        const std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Response> response);
        
    void resumeEstimator(
        const std::shared_ptr<state_estimator_msgs::srv::ResumeEstimator::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::ResumeEstimator::Response> response);
        
    void startEstimator(
        const std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Response> response);
        
    void stopEstimator(
        const std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Request> request,
        std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Response> response);

private:
    void setupServices();
};

} //namespace state_estimator

#endif
