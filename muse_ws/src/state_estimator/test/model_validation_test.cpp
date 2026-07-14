#include "state_estimator/Models/grf_contact_estimator.hpp"
#include "state_estimator/Models/leg_odometry.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace
{

const std::vector<std::string> kFootFrames{
  "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
const std::vector<std::string> kJointNames{
  "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
  "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

state_estimator::JointStateSnapshot completeJointState()
{
  state_estimator::JointStateSnapshot state;
  state.name = kJointNames;
  state.position.assign(kJointNames.size(), 0.0);
  state.velocity.assign(kJointNames.size(), 0.0);
  state.acceleration.assign(kJointNames.size(), 0.0);
  state.effort.assign(kJointNames.size(), 10.0);
  return state;
}

TEST(LegOdometryModel, AcceptsNamedJointState)
{
  state_estimator::LegOdometryModel model;
  ASSERT_TRUE(model.configure(TEST_URDF_PATH, kFootFrames, kJointNames, "base"))
    << model.lastError();

  state_estimator::LegOdometryEstimate estimate;
  const std::array<bool, 4> stance{{true, true, true, true}};
  EXPECT_TRUE(model.update(
    completeJointState(), Eigen::Vector3d::Zero(), stance,
    Eigen::Matrix3d::Identity(), estimate)) << model.lastError();
  EXPECT_EQ(estimate.stance_count, 4U);
  EXPECT_TRUE(estimate.base_velocity_world.allFinite());
}

TEST(LegOdometryModel, RejectsMissingJointData)
{
  state_estimator::LegOdometryModel model;
  ASSERT_TRUE(model.configure(TEST_URDF_PATH, kFootFrames, kJointNames, "base"));
  auto joints = completeJointState();
  joints.name.pop_back();
  joints.position.pop_back();
  joints.velocity.pop_back();

  state_estimator::LegOdometryEstimate estimate;
  EXPECT_FALSE(model.update(
    joints, Eigen::Vector3d::Zero(), {{true, true, true, true}},
    Eigen::Matrix3d::Identity(), estimate));
  EXPECT_FALSE(model.lastError().empty());
}

TEST(LegOdometryModel, ReportsInvalidFrame)
{
  state_estimator::LegOdometryModel model;
  auto frames = kFootFrames;
  frames[0] = "missing_foot";
  EXPECT_FALSE(model.configure(TEST_URDF_PATH, frames, kJointNames, "base"));
  EXPECT_NE(model.lastError().find("missing foot frame"), std::string::npos);
}

TEST(GrfContactEstimator, ConfiguresAndMapsNamedEffort)
{
  state_estimator::GrfContactEstimator model;
  const std::array<std::string, 4> frames{{
    "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}};
  const std::array<std::vector<std::string>, 4> joints{{
    {"LF_HAA", "LF_HFE", "LF_KFE"},
    {"RF_HAA", "RF_HFE", "RF_KFE"},
    {"LH_HAA", "LH_HFE", "LH_KFE"},
    {"RH_HAA", "RH_HFE", "RH_KFE"}
  }};
  ASSERT_TRUE(model.configure(
    TEST_URDF_PATH, frames, joints, 40.0, 40.0, 70.0, true,
    false, true, -1.0, true, Eigen::Vector3d::UnitZ())) << model.lastError();

  state_estimator::GrfContactEstimate estimate;
  EXPECT_TRUE(model.update(completeJointState(), estimate)) << model.lastError();
  for (double metric : estimate.metric) EXPECT_TRUE(std::isfinite(metric));
}

TEST(GrfContactEstimator, ReportsInvalidFrame)
{
  state_estimator::GrfContactEstimator model;
  const std::array<std::string, 4> frames{{
    "missing_foot", "RF_FOOT", "LH_FOOT", "RH_FOOT"}};
  const std::array<std::vector<std::string>, 4> joints{{
    {"LF_HAA", "LF_HFE", "LF_KFE"},
    {"RF_HAA", "RF_HFE", "RF_KFE"},
    {"LH_HAA", "LH_HFE", "LH_KFE"},
    {"RH_HAA", "RH_HFE", "RH_KFE"}
  }};
  EXPECT_FALSE(model.configure(
    TEST_URDF_PATH, frames, joints, 40.0, 40.0, 70.0, true,
    false, true, -1.0, true, Eigen::Vector3d::UnitZ()));
  EXPECT_NE(model.lastError().find("missing foot frame"), std::string::npos);
}

}  // namespace
