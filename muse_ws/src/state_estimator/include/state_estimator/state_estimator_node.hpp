#ifndef STATE_ESTIMATOR_NODE_HPP
#define STATE_ESTIMATOR_NODE_HPP


#include <boost/make_unique.hpp>
#include<boost/assign/list_of.hpp>
#include<boost/assert.hpp>
#include<boost/date_time/posix_time/posix_time.hpp>
#include<boost/thread/thread.hpp>
#include<boost/thread/mutex.hpp>
#include<boost/thread/shared_mutex.hpp>
#include <boost/chrono.hpp>
#include <atomic>

#include <inttypes.h>
#include <stdint.h>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <iomanip>
#include <pluginlib/class_loader.hpp>
#include <fnmatch.h>
#include <memory>
#include <stdexcept>


#include "state_estimator/plugin.hpp"
#include "state_estimator/lib.hpp"
#include "state_estimator/Robot.hpp"
#include "state_estimator/Services.hpp"





namespace state_estimator {

class state_estimator_node {
public:
    explicit state_estimator_node(rclcpp::Node::SharedPtr node);
    void shutdown();
    void time_out_thread();
    void stop_all_threads();

protected:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Robot> robot;
    pluginlib::ClassLoader<state_estimator_plugins::PluginBase> plugin_loader;
    std::vector<std::shared_ptr<state_estimator_plugins::PluginBase>> loaded_plugins;
    // ROS2 services to be added during migration


    void readParamsFromROSParameterServer();
    bool add_plugin(std::string &name);
    bool add_plugin(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist);
    bool is_blacklisted(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist);
    bool pattern_match(std::string &pattern, std::string &pl_name);
    inline bool getWhitelist(std::vector<std::string> &whitelist);
    inline bool getBlacklist(std::vector<std::string> &blacklist);
    // TODO: port ROS1 services to ROS2

};



} //namespace state_estimator

#endif
