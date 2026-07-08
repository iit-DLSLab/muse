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
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <iomanip>
#include <pluginlib/class_loader.h>
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
    state_estimator_node(ros::NodeHandle& nh) ;
    void shutdown();
    void time_out_thread();
    void stop_all_threads();

protected:
    ros::NodeHandle nh;
    std::shared_ptr<Robot> robot;
    pluginlib::ClassLoader<state_estimator_plugins::PluginBase> plugin_loader;
    std::vector<boost::shared_ptr<state_estimator_plugins::PluginBase>> loaded_plugins;
    std::vector<std::shared_ptr<ros::ServiceServer>> services;


    void readParamsFromROSParameterServer();
    bool add_plugin(std::string &name);
    bool add_plugin(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist);
    bool is_blacklisted(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist);
    bool pattern_match(std::string &pattern, std::string &pl_name);
    inline bool getWhitelist(ros::V_string &whitelist);
    inline bool getBlacklist(ros::V_string &blacklist);

    bool getActiveEstimators(state_estimator_msgs::getActiveEstimators::Request &req, state_estimator_msgs::getActiveEstimators::Response &res);
    bool getBlacklist(state_estimator_msgs::getBlacklist::Request &req, state_estimator_msgs::getBlacklist::Response &res);
    //bool getEstimatorDescription(state_estimator_msgs::getEstimatorDescription::Request &req, state_estimator_msgs::getEstimatorDescription::Response &res);
    bool getWhitelist(state_estimator_msgs::getWhitelist::Request &req, state_estimator_msgs::getWhitelist::Response &res);
    bool listAllEstimators(state_estimator_msgs::listAllEstimators::Request &req, state_estimator_msgs::listAllEstimators::Response &res);
    bool pauseEstimator(state_estimator_msgs::pauseEstimator::Request &req, state_estimator_msgs::pauseEstimator::Response &res);
    bool resetEstimator(state_estimator_msgs::resetEstimator::Request &req, state_estimator_msgs::resetEstimator::Response &res);
    bool restartEstimator(state_estimator_msgs::restartEstimator::Request &req, state_estimator_msgs::restartEstimator::Response &res);
    //bool resumeEstimator(state_estimator_msgs::resumeEstimator::Request &req, state_estimator_msgs::resumeEstimator::Response &res);
    bool startEstimator(state_estimator_msgs::startEstimator::Request &req, state_estimator_msgs::startEstimator::Response &res);
    bool stopEstimator(state_estimator_msgs::stopEstimator::Request &req, state_estimator_msgs::stopEstimator::Response &res);

};



} //namespace state_estimator

#endif
