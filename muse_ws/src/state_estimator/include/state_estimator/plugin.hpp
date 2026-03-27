#ifndef PLUGIN_HPP
#define PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "state_estimator/Robot.hpp"

namespace state_estimator_plugins {

class PluginBase {
private:
	PluginBase(const PluginBase&) = delete;

public:
	virtual ~PluginBase() {};


        void initialize(rclcpp::Node::SharedPtr node,
                        std::shared_ptr<state_estimator::Robot> robot,
                        const std::string& config_dir = "",
                        const std::string& launch_dir = "") {
            node_ = node;
            robot_ = robot;
            config_dir_ = config_dir;
            launch_dir_ = launch_dir;
            initialize_();
            paused_ = false;
            running_ = true;
            initialized_ = true;
        }

        virtual std::string getName()=0;
	virtual std::string getDescription()=0;

	void pause() {
	    if (paused_) return;
	    pause_();
	    paused_=true;
	}
	
	void shutdown() {
	    if (!running_) return;
	    shutdown_();
	    running_=false;
	    paused_=false;
	}
	
	void resume() {
	    if(!paused_) return;
	    resume_();
	    paused_ = false;
	}
	
	void reset() {
	    if (!running_) return;
	    reset_();
	}
	

	
	bool isRunning() { return running_; }
	bool isPaused() { return paused_; }
	bool isInitialized() { return initialized_; }

protected:
	PluginBase() {}
	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<state_estimator::Robot> robot_;
	std::string config_dir_;
	std::string launch_dir_;
	virtual void pause_() = 0;
	virtual void shutdown_()=0;
	virtual void resume_()=0;
        virtual void reset_()=0;
	virtual void initialize_()=0;

private:
	bool paused_;
	bool running_;
	bool initialized_;
	
};





} //namespace state_estimator_plugins

#endif
