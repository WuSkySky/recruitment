#include <gtest/gtest.h>

#include "recruitment_sim_robot_base/enable_state.hpp"
#include "recruitment_sim_robot_base/reset_request_validation.hpp"

namespace base = recruitment_sim_robot_base;

TEST(EnableState, CombinesGlobalAndComponentGates)
{
  base::EnableState state;
  EXPECT_TRUE(state.chassis_enabled());
  EXPECT_TRUE(state.gimbal_enabled());
  EXPECT_TRUE(state.shooter_enabled());

  EXPECT_TRUE(state.set(base::EnableState::SHOOTER, false));
  EXPECT_TRUE(state.chassis_enabled());
  EXPECT_TRUE(state.gimbal_enabled());
  EXPECT_FALSE(state.shooter_enabled());

  EXPECT_TRUE(state.set(base::EnableState::ALL, false));
  EXPECT_FALSE(state.chassis_enabled());
  EXPECT_FALSE(state.gimbal_enabled());
  EXPECT_FALSE(state.shooter_enabled());

  EXPECT_TRUE(state.set(base::EnableState::ALL, true));
  EXPECT_TRUE(state.chassis_enabled());
  EXPECT_TRUE(state.gimbal_enabled());
  EXPECT_FALSE(state.shooter_enabled());

  EXPECT_TRUE(state.set(base::EnableState::SHOOTER, true));
  EXPECT_TRUE(state.shooter_enabled());
}

TEST(EnableState, RejectsUnknownTargetWithoutChangingState)
{
  base::EnableState state;
  EXPECT_FALSE(state.set(255, false));
  EXPECT_TRUE(state.chassis_enabled());
  EXPECT_TRUE(state.gimbal_enabled());
  EXPECT_TRUE(state.shooter_enabled());
}

TEST(EnableState, ResetClearsComponentLocksAndSetsGlobalState)
{
  base::EnableState state;
  state.set(base::EnableState::SHOOTER, false);
  state.set(base::EnableState::CHASSIS, false);
  state.reset(true);
  EXPECT_TRUE(state.chassis_enabled());
  EXPECT_TRUE(state.gimbal_enabled());
  EXPECT_TRUE(state.shooter_enabled());
  state.reset(false);
  EXPECT_FALSE(state.chassis_enabled());
  EXPECT_FALSE(state.gimbal_enabled());
  EXPECT_FALSE(state.shooter_enabled());
}

TEST(ResetRequest, AcceptsCurrentAndNewRoundsWithSupportedColors)
{
  using Error = base::ResetRequestError;
  using Request = recruitment_sim_interfaces::srv::ResetRobot::Request;
  EXPECT_EQ(base::validate_reset_request(10, 10, Request::RED), Error::NONE);
  EXPECT_EQ(base::validate_reset_request(10, 11, Request::BLUE), Error::NONE);
  EXPECT_EQ(base::validate_reset_request(10, 11, Request::NONE), Error::NONE);
  EXPECT_EQ(base::validate_reset_request(10, 11, Request::WHITE), Error::NONE);
}

TEST(ResetRequest, RejectsStaleRoundsBeforeChangingColor)
{
  using Error = base::ResetRequestError;
  using Request = recruitment_sim_interfaces::srv::ResetRobot::Request;
  EXPECT_EQ(
    base::validate_reset_request(10, 9, Request::BLUE),
    Error::STALE_ROUND);
}

TEST(ResetRequest, RejectsUnsupportedColors)
{
  using Error = base::ResetRequestError;
  EXPECT_EQ(
    base::validate_reset_request(10, 11, 255),
    Error::INVALID_COLOR);
}
