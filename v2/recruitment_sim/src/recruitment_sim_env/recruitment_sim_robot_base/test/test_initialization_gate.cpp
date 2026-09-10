#include <gtest/gtest.h>

#include "recruitment_sim_robot_base/initialization_gate.hpp"

namespace base = recruitment_sim_robot_base;

TEST(InitializationGate, WaitsForStrictlyNewSensorSample)
{
  base::InitializationGate gate;
  EXPECT_EQ(gate.request(10, 20), base::InitializationRequestResult::SCHEDULED);
  EXPECT_TRUE(gate.blocks(20));
  EXPECT_FALSE(gate.consume_if_ready(20));
  EXPECT_FALSE(gate.blocks(21));
  EXPECT_TRUE(gate.consume_if_ready(21));
  EXPECT_FALSE(gate.consume_if_ready(22));
}

TEST(InitializationGate, SameRoundIsIdempotentAndDoesNotMoveBarrier)
{
  base::InitializationGate gate;
  EXPECT_EQ(gate.request(10, 20), base::InitializationRequestResult::SCHEDULED);
  EXPECT_EQ(gate.request(10, 30), base::InitializationRequestResult::IDEMPOTENT);
  EXPECT_TRUE(gate.consume_if_ready(21));
}

TEST(InitializationGate, RejectsOlderRound)
{
  base::InitializationGate gate;
  EXPECT_EQ(gate.request(10, 20), base::InitializationRequestResult::SCHEDULED);
  EXPECT_EQ(gate.request(9, 20), base::InitializationRequestResult::STALE);
}
