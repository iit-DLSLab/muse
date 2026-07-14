#ifndef STATE_ESTIMATOR_PLUGIN_HPP
#define STATE_ESTIMATOR_PLUGIN_HPP

#include "state_estimator/Robot.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace state_estimator_plugins
{

class PluginBase
{
public:
  PluginBase() = default;
  virtual ~PluginBase() = default;
  PluginBase(const PluginBase&) = delete;
  PluginBase& operator=(const PluginBase&) = delete;

  void initialize(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<state_estimator::Robot>& robot)
  {
    node_ = node;
    robot_ = robot;
    initialize_();
    paused_ = false;
    running_ = true;
    initialized_ = true;
  }

  virtual std::string getName() const = 0;
  virtual std::string getDescription() const = 0;

  void pause()
  {
    if (!running_ || paused_) return;
    pause_();
    paused_ = true;
  }

  void resume()
  {
    if (!running_ || !paused_) return;
    resume_();
    paused_ = false;
  }

  void reset()
  {
    if (running_) reset_();
  }

  void shutdown()
  {
    if (!running_) return;
    shutdown_();
    running_ = false;
    paused_ = false;
  }

  bool isRunning() const { return running_; }
  bool isPaused() const { return paused_; }
  bool isInitialized() const { return initialized_; }

protected:
  template<typename T>
  T parameter(const std::string& name, const T& default_value)
  {
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<T>(name, default_value);
    }
    return node_->get_parameter(name).get_value<T>();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<state_estimator::Robot> robot_;

  virtual void initialize_() = 0;
  virtual void pause_() = 0;
  virtual void resume_() = 0;
  virtual void reset_() = 0;
  virtual void shutdown_() = 0;

private:
  bool paused_{false};
  bool running_{false};
  bool initialized_{false};
};

}  // namespace state_estimator_plugins

#endif
