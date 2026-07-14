#include "state_estimator/plugin.hpp"

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include <algorithm>
#include <string>
#include <vector>

TEST(PluginLoading, DiscoversAndConstructsAllPlugins)
{
  pluginlib::ClassLoader<state_estimator_plugins::PluginBase> loader(
    "state_estimator", "state_estimator_plugins::PluginBase");
  const std::vector<std::string> expected{
    "AttitudeEstimation", "ContactDetection", "LegOdometry", "SensorFusion", "TfPublisher"};
  const auto declared = loader.getDeclaredClasses();

  for (const auto& name : expected) {
    EXPECT_NE(std::find(declared.begin(), declared.end(), name), declared.end()) << name;
    EXPECT_NO_THROW({ auto plugin = loader.createSharedInstance(name); });
  }
}
