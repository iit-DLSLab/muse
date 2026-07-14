#ifndef STATE_ESTIMATOR_NODE_HPP
#define STATE_ESTIMATOR_NODE_HPP

#include "state_estimator/Services.hpp"
#include "state_estimator/plugin.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace state_estimator
{

class StateEstimatorNode
{
public:
  explicit StateEstimatorNode(rclcpp::Node::SharedPtr node);
  ~StateEstimatorNode();

  void shutdown();

private:
  using Plugin = state_estimator_plugins::PluginBase;

  bool addPlugin(const std::string& class_name);
  bool addPlugin(
    const std::string& class_name,
    const std::vector<std::string>& blacklist,
    const std::vector<std::string>& whitelist);
  bool isBlacklisted(
    const std::string& class_name,
    const std::vector<std::string>& blacklist,
    const std::vector<std::string>& whitelist) const;
  bool patternMatches(const std::string& pattern, const std::string& value) const;
  std::vector<std::shared_ptr<Plugin>>::iterator findPlugin(const std::string& name);
  void loadConfiguredPlugins();
  void setupServices();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Robot> robot_;
  pluginlib::ClassLoader<Plugin> plugin_loader_;
  std::vector<std::shared_ptr<Plugin>> loaded_plugins_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  bool shutdown_complete_{false};
};

}  // namespace state_estimator

#endif
