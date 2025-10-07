#include "state_estimator/state_estimator_node.hpp"

#include <algorithm>
#include <functional>

namespace state_estimator {

state_estimator_node::state_estimator_node(const rclcpp::NodeOptions & options)
: Node("state_estimator", options),
  plugin_loader("state_estimator", "state_estimator_plugins::PluginBase")
{
    RCLCPP_INFO(this->get_logger(), "Constructing state_estimator_node (no plugin init here)");

    // DO NOT instantiate Robot if it's abstract. Leave nullptr or create a concrete Robot implementation.
    robot = nullptr;

    // declare the plugin list / blacklist / whitelist parameters (defaults empty)
    this->declare_parameter("plugins", std::vector<std::string>());
    this->declare_parameter("plugin_blacklist", std::vector<std::string>());
    this->declare_parameter("plugin_whitelist", std::vector<std::string>());

    // Setup ROS services (register callbacks)
    setupServices();
}

state_estimator_node::~state_estimator_node() {
    shutdown();
    stop_all_threads();
}

void state_estimator_node::init_plugins()
{
    RCLCPP_INFO(get_logger(), "Initializing plugins...");
    readParamsFromROSParameterServer();

    // Optionally start a timeout thread, if you use one
    // timeout_thread_ = std::thread(&state_estimator_node::time_out_thread, this);
}

void state_estimator_node::shutdown() {
    RCLCPP_INFO(this->get_logger(), "SE node: Shutting node down...");
    {
        std::lock_guard<std::mutex> lk(plugins_mutex_);
        for (auto const & plugin : loaded_plugins) {
            try {
                RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down " << plugin->getName() << "...");
                plugin->shutdown();
                RCLCPP_INFO(this->get_logger(), "Done");
            } catch (const std::exception &e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Exception shutting plugin: " << e.what());
            }
        }
        loaded_plugins.clear();
    }
    RCLCPP_INFO(this->get_logger(), "State Estimator shut down.");
}

void state_estimator_node::time_out_thread() {
    // Placeholder: implement your timeout logic if needed
    while (!shutdown_requested_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void state_estimator_node::stop_all_threads() {
    shutdown_requested_ = true;
    if (timeout_thread_.joinable()) {
        timeout_thread_.join();
    }
}

bool state_estimator_node::add_plugin(std::string &name) {
    // Get blacklist/whitelist from params and call overloaded add_plugin
    std::vector<std::string> blacklist, whitelist;
    getBlacklist(blacklist);
    getWhitelist(whitelist);

    // If blacklist empty and whitelist non-empty -> assume blacklist = ["*"]
    if (blacklist.empty() && !whitelist.empty()) {
        blacklist.emplace_back("*");
    }

    return add_plugin(name, blacklist, whitelist);
}

bool state_estimator_node::add_plugin(std::string &pl_name,
                                     std::vector<std::string> &blacklist,
                                     std::vector<std::string> &whitelist)
{
    if (is_blacklisted(pl_name, blacklist, whitelist)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << pl_name << " blacklisted");
        return false;
    }

    try {
        // createSharedInstance returns std::shared_ptr<PluginBase>
        auto plugin = plugin_loader.createSharedInstance(pl_name);
        RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << pl_name << " loaded");

        // === IMPORTANT: disambiguate shared_from_this() ===
        // Get a shared_ptr<state_estimator_node> from enable_shared_from_this:
        // auto derived_shared = rclcpp::Node::enable_shared_from_this<state_estimator_node>::shared_from_this();
        // Cast to rclcpp::Node::SharedPtr for the plugin interface:
        // auto node_ptr = std::static_pointer_cast<rclcpp::Node>(derived_shared);
		auto node_ptr = this->shared_from_this();

        // initialize plugin with node and robot (robot may be nullptr)
        plugin->initialize(node_ptr, robot);
        RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << pl_name << " initialized");

        {
            std::lock_guard<std::mutex> lk(plugins_mutex_);
            loaded_plugins.push_back(plugin);
        }

        return true;
    } catch (const pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Plugin " << pl_name << " load exception: " << ex.what());
    } catch (const std::exception &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Plugin " << pl_name << " load exception: " << ex.what());
    } catch (...) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Plugin " << pl_name << " load exception.");
    }
    return false;
}

inline bool state_estimator_node::getBlacklist(std::vector<std::string> &blacklist) {
    try {
        auto param = this->get_parameter("plugin_blacklist");
        blacklist = param.get_value<std::vector<std::string>>();
    } catch (const std::exception &e) {
        // If param not set, return empty vector
        blacklist.clear();
        return true;
    }
    return true;
}

inline bool state_estimator_node::getWhitelist(std::vector<std::string> &whitelist) {
    try {
        auto param = this->get_parameter("plugin_whitelist");
        whitelist = param.get_value<std::vector<std::string>>();
    } catch (const std::exception &e) {
        whitelist.clear();
        return true;
    }
    return true;
}

bool state_estimator_node::is_blacklisted(std::string &pl_name,
                                         std::vector<std::string> &blacklist,
                                         std::vector<std::string> &whitelist)
{
    // If both empty -> load all
    if (blacklist.empty() && whitelist.empty()) return false;

    // If blacklist empty but whitelist non-empty -> assume blacklist ["*"]
    if (blacklist.empty() && !whitelist.empty()) {
        blacklist.emplace_back("*");
    }

    // If pattern matches any blacklist entry and not matched in whitelist -> blacklisted
    for (auto &bl_pattern : blacklist) {
        if (pattern_match(bl_pattern, pl_name)) {
            for (auto &wl_pattern : whitelist) {
                if (pattern_match(wl_pattern, pl_name)) {
                    return false; // whitelisted
                }
            }
            return true;
        }
    }
    return false;
}

bool state_estimator_node::pattern_match(std::string &pattern, std::string &pl_name)
{
    int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
    if (cmp == 0)
        return true;
    else if (cmp == FNM_NOMATCH)
        return false;
    else {
        RCLCPP_FATAL(this->get_logger(), "Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
                     pattern.c_str(), pl_name.c_str(), cmp);
        rclcpp::shutdown();
        return false;
    }
}

void state_estimator_node::readParamsFromROSParameterServer()
{
    // Expecting a parameter "plugins" with a list of plugin names,
    // and optional "plugin_blacklist" and "plugin_whitelist".
    std::vector<std::string> plugin_names;
    std::vector<std::string> blacklist;
    std::vector<std::string> whitelist;

    try {
        this->get_parameter("plugins", plugin_names);
    } catch (...) {
        plugin_names.clear();
    }
    try {
        this->get_parameter("plugin_blacklist", blacklist);
    } catch (...) {
        blacklist.clear();
    }
    try {
        this->get_parameter("plugin_whitelist", whitelist);
    } catch (...) {
        whitelist.clear();
    }

    // If plugins list is empty, default to all declared classes
    if (plugin_names.empty()) {
        plugin_names = plugin_loader.getDeclaredClasses();
    }

    // If blacklist empty and whitelist non-empty -> assume blacklist = ["*"]
    if (blacklist.empty() && !whitelist.empty()) {
        blacklist.emplace_back("*");
    }

    for (auto &name : plugin_names) {
        std::string pl = name;
        if (!is_blacklisted(pl, blacklist, whitelist)) {
            if (!add_plugin(pl)) {
                RCLCPP_WARN(this->get_logger(), "Failed to add plugin '%s'", pl.c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Skipping blacklisted plugin '%s'", pl.c_str());
        }
    }
}

// ---------------------------
// Service handlers
// ---------------------------

void state_estimator_node::getActiveEstimators(const std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Request> req,
                                              std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Response> res)
{
    (void)req;
    std::lock_guard<std::mutex> lk(plugins_mutex_);
    for (auto const &plugin : loaded_plugins) {
        res->names.push_back(plugin->getName());
    }
}

void state_estimator_node::getBlacklistService(const std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Request> req,
                                              std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Response> res)
{
    (void)req;
    std::vector<std::string> bl;
    getBlacklist(bl);
    for (auto const &s : bl) res->names.push_back(s);
}

void state_estimator_node::getWhitelistService(const std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Request> req,
                                               std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Response> res)
{
    (void)req;
    std::vector<std::string> wl;
    getWhitelist(wl);
    for (auto const &s : wl) res->names.push_back(s);
}

void state_estimator_node::getEstimatorDescription(const std::shared_ptr<state_estimator_msgs::srv::GetEstimatorDescription::Request> req,
                                                   std::shared_ptr<state_estimator_msgs::srv::GetEstimatorDescription::Response> res)
{
    (void)req;
    std::lock_guard<std::mutex> lk(plugins_mutex_);
    for (auto const &plugin : loaded_plugins) {
        if (plugin->getName().compare(req->name) == 0) {
            res->description = plugin->getDescription();
            res->success = true;
            return;
        }
    }
    res->success = false;
}

void state_estimator_node::listAllEstimators(const std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Request> req,
                                            std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Response> res)
{
    (void)req;
    for (auto &name : plugin_loader.getDeclaredClasses()) {
        res->names.push_back(name);
    }
}

void state_estimator_node::pauseEstimator(const std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Request> req,
                                         std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Response> res)
{
    std::lock_guard<std::mutex> lk(plugins_mutex_);
    for (auto const &plugin : loaded_plugins) {
        if (plugin->getName().compare(req->name) == 0) {
            if (plugin->isPaused()) {
                res->success = false;
                return;
            }
            plugin->pause();
            res->success = true;
            return;
        }
    }
    res->success = false;
}

void state_estimator_node::resetEstimator(const std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Request> req,
                                          std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Response> res)
{
    std::lock_guard<std::mutex> lk(plugins_mutex_);
    for (auto const &plugin : loaded_plugins) {
        if (plugin->getName().compare(req->name) == 0) {
            plugin->reset();
            res->success = true;
            return;
        }
    }
    res->success = false;
}

void state_estimator_node::restartEstimator(const std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Request> req,
                                           std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Response> res)
{
    std::lock_guard<std::mutex> lk(plugins_mutex_);
    for (auto it = loaded_plugins.begin(); it != loaded_plugins.end(); ++it) {
        if ((*it)->getName().compare(req->name) == 0) {
            (*it)->shutdown();
            std::string pname = req->name;
            loaded_plugins.erase(it);
            res->success = add_plugin(pname);
            return;
        }
    }
    res->success = false;
}

void state_estimator_node::resumeEstimator(const std::shared_ptr<state_estimator_msgs::srv::ResumeEstimator::Request> req,
                                          std::shared_ptr<state_estimator_msgs::srv::ResumeEstimator::Response> res)
{
    std::lock_guard<std::mutex> lk(plugins_mutex_);
    for (auto const &plugin : loaded_plugins) {
        if (plugin->getName().compare(req->name) == 0) {
            if (!plugin->isPaused()) {
                res->success = false;
                return;
            }
            plugin->resume();
            res->success = true;
            return;
        }
    }
    res->success = false;
}

void state_estimator_node::startEstimator(const std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Request> req,
                                         std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Response> res)
{
    (void)req;
    // Start via add_plugin
    std::string name = req->name;
    res->success = add_plugin(name);
}

void state_estimator_node::stopEstimator(const std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Request> req,
                                        std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Response> res)
{
    std::lock_guard<std::mutex> lk(plugins_mutex_);
    for (auto it = loaded_plugins.begin(); it != loaded_plugins.end(); ++it) {
        if ((*it)->getName().compare(req->name) == 0) {
            (*it)->shutdown();
            loaded_plugins.erase(it);
            res->success = true;
            return;
        }
    }
    res->success = false;
}

void state_estimator_node::setupServices() {
    get_active_estimators_srv_ = this->create_service<state_estimator_msgs::srv::GetActiveEstimators>(
        "getActiveEstimators", std::bind(&state_estimator_node::getActiveEstimators, this, std::placeholders::_1, std::placeholders::_2));

    get_blacklist_srv_ = this->create_service<state_estimator_msgs::srv::GetBlacklist>(
        "getBlacklist", std::bind(&state_estimator_node::getBlacklistService, this, std::placeholders::_1, std::placeholders::_2));

    get_whitelist_srv_ = this->create_service<state_estimator_msgs::srv::GetWhitelist>(
        "getWhitelist", std::bind(&state_estimator_node::getWhitelistService, this, std::placeholders::_1, std::placeholders::_2));

    list_all_estimators_srv_ = this->create_service<state_estimator_msgs::srv::ListAllEstimators>(
        "listAllEstimators", std::bind(&state_estimator_node::listAllEstimators, this, std::placeholders::_1, std::placeholders::_2));

    pause_estimator_srv_ = this->create_service<state_estimator_msgs::srv::PauseEstimator>(
        "pauseEstimator", std::bind(&state_estimator_node::pauseEstimator, this, std::placeholders::_1, std::placeholders::_2));

    reset_estimator_srv_ = this->create_service<state_estimator_msgs::srv::ResetEstimator>(
        "resetEstimator", std::bind(&state_estimator_node::resetEstimator, this, std::placeholders::_1, std::placeholders::_2));

    restart_estimator_srv_ = this->create_service<state_estimator_msgs::srv::RestartEstimator>(
        "restartEstimator", std::bind(&state_estimator_node::restartEstimator, this, std::placeholders::_1, std::placeholders::_2));

    resume_estimator_srv_ = this->create_service<state_estimator_msgs::srv::ResumeEstimator>(
        "resumeEstimator", std::bind(&state_estimator_node::resumeEstimator, this, std::placeholders::_1, std::placeholders::_2));

    start_estimator_srv_ = this->create_service<state_estimator_msgs::srv::StartEstimator>(
        "startEstimator", std::bind(&state_estimator_node::startEstimator, this, std::placeholders::_1, std::placeholders::_2));

    stop_estimator_srv_ = this->create_service<state_estimator_msgs::srv::StopEstimator>(
        "stopEstimator", std::bind(&state_estimator_node::stopEstimator, this, std::placeholders::_1, std::placeholders::_2));

    get_estimator_description_srv_ = this->create_service<state_estimator_msgs::srv::GetEstimatorDescription>(
        "getEstimatorDescription", std::bind(&state_estimator_node::getEstimatorDescription, this, std::placeholders::_1, std::placeholders::_2));
}

} // namespace state_estimator
