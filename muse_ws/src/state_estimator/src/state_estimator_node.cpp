#include "state_estimator/state_estimator_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace state_estimator {

void state_estimator_node::shutdown() {
    RCLCPP_INFO(node_->get_logger(), "SE node: Shutting node down...");
    for (auto const &plugin : loaded_plugins) {
        RCLCPP_INFO(node_->get_logger(), "Shutting down %s...", plugin->getName().c_str());
        plugin->shutdown();
        RCLCPP_INFO(node_->get_logger(), "Done");
    }
    RCLCPP_INFO(node_->get_logger(), "State Estimator shut down.");
}

state_estimator_node::state_estimator_node(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), plugin_loader("state_estimator", "state_estimator_plugins::PluginBase") {
    // Load blacklist/whitelist parameters
    std::vector<std::string> blacklist{};
    std::vector<std::string> whitelist{};

    getBlacklist(blacklist);
    getWhitelist(whitelist);

    if (blacklist.empty() && !whitelist.empty()) {
        blacklist.emplace_back("*");
    }

    for (auto &name : plugin_loader.getDeclaredClasses()) {
        add_plugin(name, blacklist, whitelist);
    }
}

bool state_estimator_node::add_plugin(std::string &name) {
    std::vector<std::string> blacklist{}, whitelist{};
    if (!getBlacklist(blacklist)) return false;
    if (!getWhitelist(whitelist)) return false;
    if (blacklist.empty() && !whitelist.empty()) blacklist.emplace_back("*");
    return add_plugin(name, blacklist, whitelist);
}

bool state_estimator_node::add_plugin(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist) {
    if (is_blacklisted(pl_name, blacklist, whitelist)) {
        RCLCPP_INFO(node_->get_logger(), "Plugin %s blacklisted", pl_name.c_str());
        return false;
    }

    try {
        auto plugin = plugin_loader.createSharedInstance(pl_name);
        RCLCPP_INFO(node_->get_logger(), "Plugin %s loaded", pl_name.c_str());
        plugin->initialize(node_, nullptr);
        RCLCPP_INFO(node_->get_logger(), "Plugin %s initialized", pl_name.c_str());
        loaded_plugins.push_back(plugin);
        return true;
    } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception: %s", pl_name.c_str(), ex.what());
    } catch (std::exception &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception: %s", pl_name.c_str(), ex.what());
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception.", pl_name.c_str());
    }
    return false;
}

inline bool state_estimator_node::getBlacklist(std::vector<std::string> &blacklist) {
    // Declare parameter to ensure it exists
    node_->declare_parameter<std::vector<std::string>>("plugin_blacklist", blacklist);
    return node_->get_parameter("plugin_blacklist", blacklist);
}

inline bool state_estimator_node::getWhitelist(std::vector<std::string> &whitelist) {
    node_->declare_parameter<std::vector<std::string>>("plugin_whitelist", whitelist);
    return node_->get_parameter("plugin_whitelist", whitelist);
}

bool state_estimator_node::is_blacklisted(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist) {
    for (auto &bl_pattern : blacklist) {
        if (pattern_match(bl_pattern, pl_name)) {
            for (auto &wl_pattern : whitelist) {
                if (pattern_match(wl_pattern, pl_name)) return false;
            }
            return true;
        }
    }
    return false;
}

bool state_estimator_node::pattern_match(std::string &pattern, std::string &pl_name) {
    int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
    if (cmp == 0) return true;
    if (cmp != FNM_NOMATCH) {
        RCLCPP_FATAL(node_->get_logger(), "Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d", pattern.c_str(), pl_name.c_str(), cmp);
        rclcpp::shutdown();
    }
    return false;
}


//**************************************************
// Services (ROS 1 stubs removed during ROS 2 migration)
//**************************************************

} //namespace state_estimator

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("state_estimator");
	auto se = std::make_shared<state_estimator::state_estimator_node>(node);
	rclcpp::spin(node);
	se->shutdown();
	rclcpp::shutdown();
	return 0;
}

