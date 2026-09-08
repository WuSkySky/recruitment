#include <gtest/gtest.h>

#include "recruitment_sim_robot_base/enable_state.hpp"

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
