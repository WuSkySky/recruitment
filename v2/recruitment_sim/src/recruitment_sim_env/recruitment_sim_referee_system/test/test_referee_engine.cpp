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

namespace
{
constexpr std::int64_t kSecond = 1000000000LL;

/// red 被 blue 打死：两发 20 伤害，间隔 60 ms 以避开装甲 50 ms 检测间隔。
void kill_red(referee::RefereeEngine & engine, int64_t stamp, std::uint64_t first_id)
{
  engine.process_hit({"blue", first_id, "red", "armor_0", "target_collision", stamp});
  engine.process_hit(
    {"blue", first_id + 1, "red", "armor_0", "target_collision", stamp + 60000000});
}
}  // namespace

TEST(RefereeEngine, DeathResetsHeatButKeepsPermanentLock)
{
  referee::RefereeEngine engine(
    {{"red", "red", 40, 10.0, 100.0}, {"blue", "blue", 40, 88.0, 24.0}});
  for (std::uint64_t id = 1; id <= 11; ++id) {
    engine.process_shot({"red", id, "17mm", static_cast<std::int64_t>(id) * 50000000});
  }
  ASSERT_TRUE(engine.robots().at("red").permanently_locked);

  kill_red(engine, 0, 1);
  const auto & red = engine.robots().at("red");
  EXPECT_FALSE(red.alive);
  EXPECT_DOUBLE_EQ(red.heat, 0.0);
  EXPECT_FALSE(red.overheated);
  EXPECT_TRUE(red.permanently_locked);  // 3.3.1.3 的当局锁定不因死亡解除
  EXPECT_EQ(red.death_count, 1u);
  EXPECT_EQ(red.revive_ready_ns, 60000000 + 5 * kSecond);
}

TEST(RefereeEngine, RevivesAfterFiveSecondCastWithWeaknessAndInvincibility)
{
  referee::RefereeEngine engine(
    {{"red", "red", 40, 88.0, 24.0}, {"blue", "blue", 40, 88.0, 24.0}});
  kill_red(engine, 0, 1);
  ASSERT_FALSE(engine.robots().at("red").alive);
  engine.take_control_commands();

  engine.advance_time(60000000 + 5 * kSecond - 1);
  EXPECT_FALSE(engine.robots().at("red").alive);

  engine.advance_time(60000000 + 5 * kSecond);
  const auto & red = engine.robots().at("red");
  EXPECT_TRUE(red.alive);
  EXPECT_EQ(red.current_hp, 8);  // 上限的 20%
  EXPECT_TRUE(red.weakened);
  EXPECT_TRUE(red.invincible);
  EXPECT_EQ(red.invincible_until_ns, 60000000 + 5 * kSecond + 30 * kSecond);

  const auto commands = engine.take_control_commands();
  ASSERT_EQ(commands.size(), 2u);
  EXPECT_EQ(commands[0].target, referee::ControlCommand::ALL);
  EXPECT_TRUE(commands[0].enabled);
  EXPECT_EQ(commands[1].target, referee::ControlCommand::SHOOTER);
  EXPECT_FALSE(commands[1].enabled);
}

TEST(RefereeEngine, ReviveCastGrowsByFiveSecondsPerDeath)
{
  referee::RefereeEngine engine(
    {{"red", "red", 40, 88.0, 24.0}, {"blue", "blue", 40, 88.0, 24.0}});
  kill_red(engine, 0, 1);
  engine.advance_time(60000000 + 5 * kSecond);
  ASSERT_TRUE(engine.robots().at("red").alive);

  // 进己方补给区解除虚弱与无敌，然后再次击杀
  engine.supply_tick({"red"}, kSecond);
  ASSERT_FALSE(engine.robots().at("red").weakened);
  ASSERT_FALSE(engine.robots().at("red").invincible);

  const int64_t second_death = 60000000 + 5 * kSecond + kSecond;
  kill_red(engine, second_death, 3);
  EXPECT_FALSE(engine.robots().at("red").alive);
  EXPECT_EQ(engine.robots().at("red").death_count, 2u);
  EXPECT_EQ(engine.robots().at("red").revive_ready_ns, second_death + 10 * kSecond);

  engine.advance_time(second_death + 10 * kSecond - 1);
  EXPECT_FALSE(engine.robots().at("red").alive);
  engine.advance_time(second_death + 10 * kSecond);
  EXPECT_TRUE(engine.robots().at("red").alive);
}

TEST(RefereeEngine, InvincibleTargetIgnoresDamage)
{
  referee::RefereeEngine engine(
    {{"red", "red", 40, 88.0, 24.0}, {"blue", "blue", 40, 88.0, 24.0}});
  kill_red(engine, 0, 1);
  engine.advance_time(60000000 + 5 * kSecond);
  ASSERT_TRUE(engine.robots().at("red").invincible);

  const int hp = engine.robots().at("red").current_hp;
  const auto hits = engine.robots().at("blue").total_hits;
  const auto damage = engine.robots().at("blue").attack_damage;
  EXPECT_FALSE(engine.process_hit(
    {"blue", 9, "red", "armor_0", "target_collision", 60000000 + 5 * kSecond + kSecond}));
  EXPECT_EQ(engine.robots().at("red").current_hp, hp);
  EXPECT_TRUE(engine.robots().at("red").alive);
  EXPECT_EQ(engine.robots().at("blue").total_hits, hits);
  EXPECT_EQ(engine.robots().at("blue").attack_damage, damage);
}

TEST(RefereeEngine, InvincibilityExpiresButWeaknessPersists)
{
  referee::RefereeEngine engine(
    {{"red", "red", 40, 88.0, 24.0}, {"blue", "blue", 40, 88.0, 24.0}});
  kill_red(engine, 0, 1);
  const int64_t revived = 60000000 + 5 * kSecond;
  engine.advance_time(revived);

  engine.advance_time(revived + 30 * kSecond - 1);
  EXPECT_TRUE(engine.robots().at("red").invincible);
  engine.advance_time(revived + 30 * kSecond);
  EXPECT_FALSE(engine.robots().at("red").invincible);
  EXPECT_TRUE(engine.robots().at("red").weakened);  // 只能靠补给区解除
}

TEST(RefereeEngine, SupplyZoneClearsWeaknessAndHealsAtQuarterMaxPerSecond)
{
  referee::RefereeEngine engine(
    {{"red", "red", 40, 88.0, 24.0}, {"blue", "blue", 40, 88.0, 24.0}});
  kill_red(engine, 0, 1);
  engine.advance_time(60000000 + 5 * kSecond);
  ASSERT_TRUE(engine.robots().at("red").weakened);
  ASSERT_EQ(engine.robots().at("red").current_hp, 8);
  engine.take_control_commands();

  engine.supply_tick({"red"}, kSecond);
  const auto & red = engine.robots().at("red");
  EXPECT_FALSE(red.weakened);
  EXPECT_FALSE(red.invincible);
  EXPECT_EQ(red.current_hp, 18);  // 8 + 25% * 40

  const auto commands = engine.take_control_commands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].target, referee::ControlCommand::SHOOTER);
  EXPECT_TRUE(commands[0].enabled);
}

TEST(RefereeEngine, HealingCapsAtMaxHpAndSkipsDeadRobots)
{
  referee::RefereeEngine engine(
    {{"red", "red", 40, 88.0, 24.0}, {"blue", "blue", 40, 88.0, 24.0}});
  engine.supply_tick({"red"}, 10 * kSecond);
  EXPECT_EQ(engine.robots().at("red").current_hp, 40);

  kill_red(engine, 0, 1);
  ASSERT_FALSE(engine.robots().at("red").alive);
  engine.supply_tick({"red"}, 10 * kSecond);
  EXPECT_FALSE(engine.robots().at("red").alive);
  EXPECT_EQ(engine.robots().at("red").current_hp, 0);
}
