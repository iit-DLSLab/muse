#include "state_estimator/state_estimator_node.hpp"

#include <fnmatch.h>

#include <algorithm>
#include <functional>
#include <utility>

namespace state_estimator
{

StateEstimatorNode::StateEstimatorNode(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  plugin_loader_("state_estimator", "state_estimator_plugins::PluginBase")
{
  if (!node_->has_parameter("plugin_blacklist")) {
    node_->declare_parameter<std::vector<std::string>>(
      "plugin_blacklist", std::vector<std::string>{});
  }
  if (!node_->has_parameter("plugin_whitelist")) {
    node_->declare_parameter<std::vector<std::string>>(
      "plugin_whitelist", std::vector<std::string>{});
  }

  setupServices();
  loadConfiguredPlugins();
}

StateEstimatorNode::~StateEstimatorNode()
{
  shutdown();
}

void StateEstimatorNode::shutdown()
{
  if (shutdown_complete_) return;

  RCLCPP_INFO(node_->get_logger(), "Shutting down state estimator plugins");
  for (const auto& plugin : loaded_plugins_) {
    try {
      plugin->shutdown();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Error shutting down %s: %s",
        plugin->getName().c_str(), error.what());
    }
  }
  loaded_plugins_.clear();
  shutdown_complete_ = true;
}

void StateEstimatorNode::loadConfiguredPlugins()
{
  auto blacklist = node_->get_parameter("plugin_blacklist").as_string_array();
  auto whitelist = node_->get_parameter("plugin_whitelist").as_string_array();
  if (blacklist.empty() && !whitelist.empty()) blacklist.emplace_back("*");

  for (const auto& class_name : plugin_loader_.getDeclaredClasses()) {
    addPlugin(class_name, blacklist, whitelist);
  }
}

bool StateEstimatorNode::addPlugin(const std::string& class_name)
{
  auto blacklist = node_->get_parameter("plugin_blacklist").as_string_array();
  auto whitelist = node_->get_parameter("plugin_whitelist").as_string_array();
  if (blacklist.empty() && !whitelist.empty()) blacklist.emplace_back("*");
  return addPlugin(class_name, blacklist, whitelist);
}

bool StateEstimatorNode::addPlugin(
  const std::string& class_name,
  const std::vector<std::string>& blacklist,
  const std::vector<std::string>& whitelist)
{
  if (findPlugin(class_name) != loaded_plugins_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Plugin %s is already loaded", class_name.c_str());
    return false;
  }
  if (isBlacklisted(class_name, blacklist, whitelist)) {
    RCLCPP_INFO(node_->get_logger(), "Plugin %s is blacklisted", class_name.c_str());
    return false;
  }

  try {
    auto plugin = plugin_loader_.createSharedInstance(class_name);
    plugin->initialize(node_, robot_);
    RCLCPP_INFO(node_->get_logger(), "Plugin %s initialized", class_name.c_str());
    loaded_plugins_.push_back(std::move(plugin));
    return true;
  } catch (const pluginlib::PluginlibException& error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Plugin %s load exception: %s",
      class_name.c_str(), error.what());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Plugin %s initialization exception: %s",
      class_name.c_str(), error.what());
  }
  return false;
}

bool StateEstimatorNode::isBlacklisted(
  const std::string& class_name,
  const std::vector<std::string>& blacklist,
  const std::vector<std::string>& whitelist) const
{
  for (const auto& pattern : blacklist) {
    if (!patternMatches(pattern, class_name)) continue;
    return std::none_of(
      whitelist.begin(), whitelist.end(),
      [this, &class_name](const std::string& allowed) {
        return patternMatches(allowed, class_name);
      });
  }
  return false;
}

bool StateEstimatorNode::patternMatches(
  const std::string& pattern, const std::string& value) const
{
  const int result = fnmatch(pattern.c_str(), value.c_str(), FNM_CASEFOLD);
  if (result == 0) return true;
  if (result == FNM_NOMATCH) return false;
  RCLCPP_ERROR(
    node_->get_logger(), "fnmatch failed for '%s' and '%s'",
    pattern.c_str(), value.c_str());
  return false;
}

std::vector<std::shared_ptr<StateEstimatorNode::Plugin>>::iterator
StateEstimatorNode::findPlugin(const std::string& name)
{
  return std::find_if(
    loaded_plugins_.begin(), loaded_plugins_.end(),
    [&name](const std::shared_ptr<Plugin>& plugin) {
      return plugin->getName() == name;
    });
}

void StateEstimatorNode::setupServices()
{
  using namespace std::placeholders;
  namespace srv = state_estimator_msgs::srv;

  services_.push_back(node_->create_service<srv::GetActiveEstimators>(
    "~/get_active_estimators",
    [this](const std::shared_ptr<srv::GetActiveEstimators::Request>,
           std::shared_ptr<srv::GetActiveEstimators::Response> response) {
      for (const auto& plugin : loaded_plugins_) response->names.push_back(plugin->getName());
    }));

  services_.push_back(node_->create_service<srv::GetBlacklist>(
    "~/get_blacklist",
    [this](const std::shared_ptr<srv::GetBlacklist::Request>,
           std::shared_ptr<srv::GetBlacklist::Response> response) {
      response->names = node_->get_parameter("plugin_blacklist").as_string_array();
    }));

  services_.push_back(node_->create_service<srv::GetWhitelist>(
    "~/get_whitelist",
    [this](const std::shared_ptr<srv::GetWhitelist::Request>,
           std::shared_ptr<srv::GetWhitelist::Response> response) {
      response->names = node_->get_parameter("plugin_whitelist").as_string_array();
    }));

  services_.push_back(node_->create_service<srv::ListAllEstimators>(
    "~/list_all_estimators",
    [this](const std::shared_ptr<srv::ListAllEstimators::Request>,
           std::shared_ptr<srv::ListAllEstimators::Response> response) {
      response->names = plugin_loader_.getDeclaredClasses();
    }));

  services_.push_back(node_->create_service<srv::GetEstimatorDescription>(
    "~/get_estimator_description",
    [this](const std::shared_ptr<srv::GetEstimatorDescription::Request> request,
           std::shared_ptr<srv::GetEstimatorDescription::Response> response) {
      const auto plugin = findPlugin(request->name);
      response->success = plugin != loaded_plugins_.end();
      if (response->success) response->description = (*plugin)->getDescription();
    }));

  services_.push_back(node_->create_service<srv::PauseEstimator>(
    "~/pause_estimator",
    [this](const std::shared_ptr<srv::PauseEstimator::Request> request,
           std::shared_ptr<srv::PauseEstimator::Response> response) {
      const auto plugin = findPlugin(request->name);
      response->success = plugin != loaded_plugins_.end() && !(*plugin)->isPaused();
      if (response->success) (*plugin)->pause();
    }));

  services_.push_back(node_->create_service<srv::ResumeEstimator>(
    "~/resume_estimator",
    [this](const std::shared_ptr<srv::ResumeEstimator::Request> request,
           std::shared_ptr<srv::ResumeEstimator::Response> response) {
      const auto plugin = findPlugin(request->name);
      response->success = plugin != loaded_plugins_.end() && (*plugin)->isPaused();
      if (response->success) (*plugin)->resume();
    }));

  services_.push_back(node_->create_service<srv::ResetEstimator>(
    "~/reset_estimator",
    [this](const std::shared_ptr<srv::ResetEstimator::Request> request,
           std::shared_ptr<srv::ResetEstimator::Response> response) {
      const auto plugin = findPlugin(request->name);
      response->success = plugin != loaded_plugins_.end();
      if (response->success) (*plugin)->reset();
    }));

  services_.push_back(node_->create_service<srv::StartEstimator>(
    "~/start_estimator",
    [this](const std::shared_ptr<srv::StartEstimator::Request> request,
           std::shared_ptr<srv::StartEstimator::Response> response) {
      response->success = addPlugin(request->name);
    }));

  services_.push_back(node_->create_service<srv::StopEstimator>(
    "~/stop_estimator",
    [this](const std::shared_ptr<srv::StopEstimator::Request> request,
           std::shared_ptr<srv::StopEstimator::Response> response) {
      const auto plugin = findPlugin(request->name);
      response->success = plugin != loaded_plugins_.end();
      if (response->success) {
        (*plugin)->shutdown();
        loaded_plugins_.erase(plugin);
      }
    }));

  services_.push_back(node_->create_service<srv::RestartEstimator>(
    "~/restart_estimator",
    [this](const std::shared_ptr<srv::RestartEstimator::Request> request,
           std::shared_ptr<srv::RestartEstimator::Response> response) {
      const auto plugin = findPlugin(request->name);
      if (plugin == loaded_plugins_.end()) {
        response->success = false;
        return;
      }
      (*plugin)->shutdown();
      loaded_plugins_.erase(plugin);
      response->success = addPlugin(request->name);
    }));
}

}  // namespace state_estimator
