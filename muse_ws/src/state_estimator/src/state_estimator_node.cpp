#include "state_estimator/state_estimator_node.hpp"
#include <yaml-cpp/yaml.h>
#include <fnmatch.h>

namespace state_estimator {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void state_estimator_node::shutdown() {
    RCLCPP_INFO(node_->get_logger(), "SE node: Shutting node down...");
    for (auto const& plugin : loaded_plugins_) {
        RCLCPP_INFO(node_->get_logger(), "Shutting down %s...", plugin->getName().c_str());
        plugin->shutdown();
        RCLCPP_INFO(node_->get_logger(), "Done");
    }
    RCLCPP_INFO(node_->get_logger(), "State Estimator shut down.");
}

state_estimator_node::state_estimator_node(rclcpp::Node::SharedPtr node)
    : node_(node),
      plugin_loader_("state_estimator", "state_estimator_plugins::PluginBase")
{
    node_->declare_parameter("config_dir", "");
    node_->declare_parameter("launch_dir", "");
    config_dir_ = node_->get_parameter("config_dir").as_string();
    launch_dir_ = node_->get_parameter("launch_dir").as_string();

    RCLCPP_INFO(node_->get_logger(), "config_dir: %s", config_dir_.c_str());
    RCLCPP_INFO(node_->get_logger(), "launch_dir: %s", launch_dir_.c_str());

    std::vector<std::string> blacklist{}, whitelist{};
    getBlacklist(blacklist);
    getWhitelist(whitelist);

    if (blacklist.empty() && !whitelist.empty())
        blacklist.emplace_back("*");

    for (auto& name : plugin_loader_.getDeclaredClasses()) {
        add_plugin(name, blacklist, whitelist);
    }

    using namespace std::placeholders;
    services_.push_back(node_->create_service<state_estimator_msgs::srv::GetActiveEstimators>(
        "getActiveEstimators",
        std::bind(&state_estimator_node::getActiveEstimatorsSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::GetBlacklist>(
        "getBlacklist",
        std::bind(&state_estimator_node::getBlacklistSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::GetWhitelist>(
        "getWhitelist",
        std::bind(&state_estimator_node::getWhitelistSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::ListAllEstimators>(
        "listAllEstimators",
        std::bind(&state_estimator_node::listAllEstimatorsSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::PauseEstimator>(
        "pauseEstimator",
        std::bind(&state_estimator_node::pauseEstimatorSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::ResetEstimator>(
        "resetEstimator",
        std::bind(&state_estimator_node::resetEstimatorSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::RestartEstimator>(
        "restartEstimator",
        std::bind(&state_estimator_node::restartEstimatorSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::StartEstimator>(
        "startEstimator",
        std::bind(&state_estimator_node::startEstimatorSrv, this, _1, _2)));
    services_.push_back(node_->create_service<state_estimator_msgs::srv::StopEstimator>(
        "stopEstimator",
        std::bind(&state_estimator_node::stopEstimatorSrv, this, _1, _2)));
}

// ---------------------------------------------------------------------------
// Plugin management
// ---------------------------------------------------------------------------

bool state_estimator_node::add_plugin(const std::string& name) {
    std::vector<std::string> blacklist{}, whitelist{};
    if (!getBlacklist(blacklist)) return false;
    if (!getWhitelist(whitelist)) return false;
    if (blacklist.empty() && !whitelist.empty())
        blacklist.emplace_back("*");
    return add_plugin(name, blacklist, whitelist);
}

bool state_estimator_node::add_plugin(const std::string& pl_name,
                                       const std::vector<std::string>& blacklist,
                                       const std::vector<std::string>& whitelist) {
    if (is_blacklisted(pl_name, blacklist, whitelist)) {
        RCLCPP_INFO(node_->get_logger(), "Plugin %s blacklisted", pl_name.c_str());
        return false;
    }
    try {
        auto plugin = plugin_loader_.createSharedInstance(pl_name);
        RCLCPP_INFO(node_->get_logger(), "Plugin %s loaded", pl_name.c_str());
        plugin->initialize(node_, nullptr, config_dir_, launch_dir_);
        RCLCPP_INFO(node_->get_logger(), "Plugin %s initialized", pl_name.c_str());
        loaded_plugins_.push_back(plugin);
        return true;
    } catch (pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception: %s", pl_name.c_str(), ex.what());
    } catch (std::exception& ex) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception: %s", pl_name.c_str(), ex.what());
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s unknown load exception.", pl_name.c_str());
    }
    return false;
}

bool state_estimator_node::getBlacklist(std::vector<std::string>& blacklist) {
    if (launch_dir_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "launch_dir not set; cannot load blacklist.");
        return false;
    }
    try {
        YAML::Node cfg = YAML::LoadFile(launch_dir_ + "/pluginlist.yaml");
        if (cfg["plugin_blacklist"])
            blacklist = cfg["plugin_blacklist"].as<std::vector<std::string>>();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Could not load pluginlist.yaml: %s", e.what());
        return false;
    }
}

bool state_estimator_node::getWhitelist(std::vector<std::string>& whitelist) {
    if (launch_dir_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "launch_dir not set; cannot load whitelist.");
        return false;
    }
    try {
        YAML::Node cfg = YAML::LoadFile(launch_dir_ + "/pluginlist.yaml");
        if (cfg["plugin_whitelist"])
            whitelist = cfg["plugin_whitelist"].as<std::vector<std::string>>();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Could not load pluginlist.yaml: %s", e.what());
        return false;
    }
}

bool state_estimator_node::is_blacklisted(const std::string& pl_name,
                                           const std::vector<std::string>& blacklist,
                                           const std::vector<std::string>& whitelist) {
    for (const auto& bl_pattern : blacklist) {
        if (pattern_match(bl_pattern, pl_name)) {
            for (const auto& wl_pattern : whitelist) {
                if (pattern_match(wl_pattern, pl_name))
                    return false;
            }
            return true;
        }
    }
    return false;
}

bool state_estimator_node::pattern_match(const std::string& pattern, const std::string& pl_name) {
    int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
    if (cmp == 0)
        return true;
    if (cmp != FNM_NOMATCH) {
        RCLCPP_FATAL(node_->get_logger(),
            "Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
            pattern.c_str(), pl_name.c_str(), cmp);
        rclcpp::shutdown();
    }
    return false;
}

// ---------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------

void state_estimator_node::getActiveEstimatorsSrv(
    const std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Request> /*req*/,
    std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Response> res)
{
    for (auto const& plugin : loaded_plugins_)
        res->names.push_back(plugin->getName());
}

void state_estimator_node::getBlacklistSrv(
    const std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Request> /*req*/,
    std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Response> res)
{
    std::vector<std::string> blacklist{};
    getBlacklist(blacklist);
    for (auto const& name : blacklist)
        res->names.push_back(name);
}

void state_estimator_node::getWhitelistSrv(
    const std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Request> /*req*/,
    std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Response> res)
{
    std::vector<std::string> whitelist{};
    getWhitelist(whitelist);
    for (auto const& name : whitelist)
        res->names.push_back(name);
}

void state_estimator_node::listAllEstimatorsSrv(
    const std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Request> /*req*/,
    std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Response> res)
{
    for (auto& name : plugin_loader_.getDeclaredClasses())
        res->names.push_back(name);
}

void state_estimator_node::pauseEstimatorSrv(
    const std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Request> req,
    std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Response> res)
{
    for (auto const& plugin : loaded_plugins_) {
        if (plugin->getName() == req->name) {
            if (plugin->isPaused()) {
                RCLCPP_INFO(node_->get_logger(), "Plugin %s already paused.", req->name.c_str());
                res->success = false;
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Pausing %s...", req->name.c_str());
            plugin->pause();
            RCLCPP_INFO(node_->get_logger(), "Plugin %s paused.", req->name.c_str());
            res->success = true;
            return;
        }
    }
    RCLCPP_WARN(node_->get_logger(), "Plugin %s not loaded.", req->name.c_str());
    res->success = false;
}

void state_estimator_node::resetEstimatorSrv(
    const std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Request> req,
    std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Response> res)
{
    for (auto const& plugin : loaded_plugins_) {
        if (plugin->getName() == req->name) {
            RCLCPP_INFO(node_->get_logger(), "Resetting %s...", req->name.c_str());
            plugin->reset();
            RCLCPP_INFO(node_->get_logger(), "Plugin %s reset.", req->name.c_str());
            res->success = true;
            return;
        }
    }
    RCLCPP_WARN(node_->get_logger(), "Plugin %s not loaded.", req->name.c_str());
    res->success = false;
}

void state_estimator_node::restartEstimatorSrv(
    const std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Request> req,
    std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Response> res)
{
    for (auto const& plugin : loaded_plugins_) {
        if (plugin->getName() == req->name) {
            RCLCPP_INFO(node_->get_logger(), "Shutting down %s...", req->name.c_str());
            plugin->shutdown();
            RCLCPP_INFO(node_->get_logger(), "Plugin %s shut down", req->name.c_str());
            res->success = add_plugin(req->name);
            return;
        }
    }
    RCLCPP_WARN(node_->get_logger(), "Plugin %s not loaded.", req->name.c_str());
    res->success = false;
}

void state_estimator_node::startEstimatorSrv(
    const std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Request> req,
    std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Response> res)
{
    res->success = add_plugin(req->name);
}

void state_estimator_node::stopEstimatorSrv(
    const std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Request> req,
    std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Response> res)
{
    for (auto const& plugin : loaded_plugins_) {
        if (plugin->getName() == req->name) {
            if (!plugin->isRunning()) {
                RCLCPP_INFO(node_->get_logger(), "Plugin %s is not running.", req->name.c_str());
                res->success = false;
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Shutting down %s...", req->name.c_str());
            plugin->shutdown();
            RCLCPP_INFO(node_->get_logger(), "Plugin %s shut down", req->name.c_str());
            res->success = true;
            return;
        }
    }
    RCLCPP_WARN(node_->get_logger(), "Plugin %s not loaded.", req->name.c_str());
    res->success = false;
}

} // namespace state_estimator
