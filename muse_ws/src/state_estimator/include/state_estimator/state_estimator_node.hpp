#ifndef STATE_ESTIMATOR_NODE_HPP
#define STATE_ESTIMATOR_NODE_HPP

#include <atomic>
#include <fnmatch.h>
#include <inttypes.h>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "state_estimator/plugin.hpp"
#include "state_estimator/lib.hpp"
#include "state_estimator/Robot.hpp"
#include "state_estimator/Services.hpp"


namespace state_estimator {

class state_estimator_node {
public:
    explicit state_estimator_node(rclcpp::Node::SharedPtr node);
    void shutdown();

protected:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Robot> robot_;
    pluginlib::ClassLoader<state_estimator_plugins::PluginBase> plugin_loader_;
    std::vector<std::shared_ptr<state_estimator_plugins::PluginBase>> loaded_plugins_;
    std::vector<rclcpp::ServiceBase::SharedPtr> services_;
    std::string config_dir_;
    std::string launch_dir_;

    bool add_plugin(const std::string& name);
    bool add_plugin(const std::string& name,
                    const std::vector<std::string>& blacklist,
                    const std::vector<std::string>& whitelist);
    bool is_blacklisted(const std::string& name,
                        const std::vector<std::string>& blacklist,
                        const std::vector<std::string>& whitelist);
    bool pattern_match(const std::string& pattern, const std::string& name);
    bool getBlacklist(std::vector<std::string>& blacklist);
    bool getWhitelist(std::vector<std::string>& whitelist);

    void getActiveEstimatorsSrv(
        const std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Response> res);
    void getBlacklistSrv(
        const std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Response> res);
    void getWhitelistSrv(
        const std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Response> res);
    void listAllEstimatorsSrv(
        const std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Response> res);
    void pauseEstimatorSrv(
        const std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Response> res);
    void resetEstimatorSrv(
        const std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Response> res);
    void restartEstimatorSrv(
        const std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Response> res);
    void startEstimatorSrv(
        const std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Response> res);
    void stopEstimatorSrv(
        const std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Request> req,
        std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Response> res);
};


} //namespace state_estimator

#endif
