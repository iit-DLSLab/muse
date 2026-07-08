// include/state_estimator/plugin_base.hpp
#pragma once
#include <ros/ros.h>
#include <memory>

namespace state_estimator_plugins {

class PluginBase {
public:
  virtual ~PluginBase() = default;
  virtual void initialize(ros::NodeHandle& nh, std::shared_ptr<void> robot) = 0;
  virtual void pause() = 0;
  virtual void shutdown() = 0;
  virtual std::string getName() = 0;
  virtual bool isRunning() = 0;
  virtual bool isPaused() = 0;
};

} // namespace state_estimator_plugins
