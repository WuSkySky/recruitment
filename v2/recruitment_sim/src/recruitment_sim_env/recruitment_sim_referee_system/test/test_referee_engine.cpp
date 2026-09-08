#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "recruitment_sim_referee_system/referee_engine.hpp"

namespace referee = recruitment_sim_referee_system;

TEST(RefereeEngine, CountsShotsHeatsAndCoolsAtTenHertz)
{
  referee::RefereeEngine engine({{"red", "red", 350, 88.0, 24.0}});
  EXPECT_TRUE(engine.process_shot({"red", 1, "17mm", 0}));
  EXPECT_TRUE(engine.process_shot({"red", 2, "17mm", 50000000}));
  EXPECT_FALSE(engine.process_shot({"red", 3, "42mm", 100000000}));

  engine.cool_one_period();
  const auto & state = engine.robots().at("red");
  EXPECT_EQ(state.shots_last_period, 2u);
  EXPECT_EQ(state.total_shots, 2u);
  EXPECT_DOUBLE_EQ(state.heat, 17.6);

  for (int i = 0; i < 20; ++i) {
    engine.cool_one_period();
  }
  EXPECT_DOUBLE_EQ(engine.robots().at("red").heat, 0.0);
}

TEST(RefereeEngine, LocksAboveLimitAndUnlocksOnlyAtZero)
{
  referee::RefereeEngine engine({{"red", "red", 350, 18.0, 10.0}});
  engine.process_shot({"red", 1, "17mm", 0});
  engine.process_shot({"red", 2, "17mm", 50000000});

  auto commands = engine.take_control_commands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].target, referee::ControlCommand::SHOOTER);
  EXPECT_FALSE(commands[0].enabled);
  EXPECT_TRUE(engine.robots().at("red").overheated);

  for (int i = 0; i < 19; ++i) {
    engine.cool_one_period();
  }
  EXPECT_TRUE(engine.take_control_commands().empty());
  engine.cool_one_period();
  commands = engine.take_control_commands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_TRUE(commands[0].enabled);
  EXPECT_FALSE(engine.robots().at("red").overheated);
}

TEST(RefereeEngine, PermanentlyLocksAtLimitPlusOneHundred)
{
  referee::RefereeEngine engine({{"red", "red", 350, 10.0, 100.0}});
  for (std::uint64_t id = 1; id <= 11; ++id) {
    engine.process_shot({"red", id, "17mm", static_cast<std::int64_t>(id) * 50000000});
  }
  EXPECT_TRUE(engine.robots().at("red").permanently_locked);
  engine.take_control_commands();
  for (int i = 0; i < 20; ++i) {
    engine.cool_one_period();
  }
  EXPECT_DOUBLE_EQ(engine.robots().at("red").heat, 0.0);
  EXPECT_TRUE(engine.robots().at("red").overheated);
  EXPECT_TRUE(engine.take_control_commands().empty());
}

TEST(RefereeEngine, AppliesEnemyArmorDamageAndDetectionInterval)
{
  referee::RefereeEngine engine({
    {"red", "red", 40, 88.0, 24.0},
    {"red_friend", "red", 40, 88.0, 24.0},
    {"blue", "blue", 40, 88.0, 24.0},
  });

  EXPECT_FALSE(engine.process_hit({"red", 1, "red_friend", "armor_0", "target_collision", 0}));
  EXPECT_FALSE(engine.process_hit({"red", 2, "blue", "chassis", "collision", 0}));
  EXPECT_TRUE(engine.process_hit({"red", 3, "blue", "armor_0", "target_collision", 0}));
  EXPECT_FALSE(engine.process_hit({"red", 3, "blue", "armor_0", "target_collision", 60000000}));
  EXPECT_FALSE(engine.process_hit({"red", 4, "blue", "armor_0", "target_collision", 20000000}));
  EXPECT_TRUE(engine.process_hit({"red", 5, "blue", "armor_0", "target_collision", 60000000}));

  EXPECT_EQ(engine.robots().at("blue").current_hp, 0);
  EXPECT_FALSE(engine.robots().at("blue").alive);
  EXPECT_EQ(engine.robots().at("red").total_hits, 2u);
  const auto commands = engine.take_control_commands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].robot_name, "blue");
  EXPECT_EQ(commands[0].target, referee::ControlCommand::ALL);
  EXPECT_FALSE(commands[0].enabled);
}

TEST(RefereeEngine, RejectsInvalidConfiguration)
{
  EXPECT_THROW(referee::RefereeEngine({}), std::invalid_argument);
  EXPECT_THROW(
    referee::RefereeEngine({{"red", "red", 0, 88.0, 24.0}}), std::invalid_argument);
}
