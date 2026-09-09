#include <gtest/gtest.h>
#include "recruitment_sim_referee_system/match_engine.hpp"
namespace r = recruitment_sim_referee_system;
constexpr int64_t sec = 1000000000LL;
static std::vector<r::RobotConfig> robots(int hp = 40)
{
  return {{"r", "red", hp, 88, 24}, {"r2", "red", hp, 88, 24}, {"b", "blue", hp, 88, 24}};
}
static r::MatchFrame frame(int64_t t, std::vector<r::RobotPosition> positions = {})
{
  r::MatchFrame f; f.round = 1; f.stamp = t; f.positions = positions; return f;
}
static r::HitEvent hit(const std::string & a, const std::string & b, int id, int64_t t)
{
  return {a, static_cast<uint64_t>(id), b, "armor_0", "target_collision", t};
}
static void run(r::MatchEngine & match, uint64_t round = 1, int64_t stamp = 0)
{
  ASSERT_TRUE(match.reset(round));
  match.reset_complete();
  ASSERT_EQ(match.state(), r::MatchEngine::READY);
  ASSERT_TRUE(match.start(stamp));
}
TEST(Match, TrainingAndRoundReset)
{
  r::MatchEngine m(robots());
  auto f = frame(sec); f.round = 0; f.positions = {{"r", 0, 0}};
  f.hits = {hit("r", "b", 1, sec)};
  for (int i = 0; i < 20; ++i) {f.shots.push_back({"r", uint64_t(i), "17mm", sec});}
  m.process(f);
  EXPECT_EQ(m.referee().robots().at("b").current_hp, 20);
  EXPECT_EQ(m.points()[1], 200); EXPECT_EQ(m.owner(), -1);
  EXPECT_FALSE(m.start(0));
  ASSERT_TRUE(m.reset(1)); EXPECT_FALSE(m.reset(2)); m.reset_complete();
  EXPECT_EQ(m.state(), r::MatchEngine::READY); EXPECT_TRUE(m.start(2 * sec));
  EXPECT_EQ(m.referee().robots().at("b").current_hp, 40);
  EXPECT_FALSE(m.referee().robots().at("r").permanently_locked);
  EXPECT_EQ(m.damage()[0], 0u); EXPECT_EQ(m.elapsed(), 0);
  m.process(f); EXPECT_EQ(m.referee().robots().at("b").current_hp, 40);
  EXPECT_FALSE(m.reset(2));
  m.end(); m.paused();
  EXPECT_TRUE(m.reset(2)); m.reset_complete(); EXPECT_TRUE(m.start(3 * sec)); EXPECT_EQ(m.round(), 2u);
}
TEST(Match, BoundaryFirstArrivalAndNoStacking)
{
  r::MatchEngine m(robots()); run(m);
  m.process(frame(1, {{"r", -1.5, 1.5}, {"r2", 1.5, -1.5}}));
  EXPECT_EQ(m.owner(), 0);
  m.process(frame(sec + 1, {{"r", 0, 0}, {"r2", 0, 0}, {"b", 0, 0}}));
  EXPECT_EQ(m.points()[1], 199); EXPECT_EQ(m.points()[0], 200);
  EXPECT_EQ(m.owner(), 0);
}
TEST(Match, StartDoesNotResetReadyRobotState)
{
  r::MatchEngine m(robots());
  ASSERT_TRUE(m.reset(1)); m.reset_complete();
  auto f = frame(sec);
  f.shots = {{"r", 1, "17mm", sec}};
  f.hits = {hit("r", "b", 1, sec)};
  m.process(f);
  EXPECT_EQ(m.referee().robots().at("b").current_hp, 20);
  EXPECT_DOUBLE_EQ(m.referee().robots().at("r").heat, 10.0);
  EXPECT_EQ(m.points()[1], 200);
  ASSERT_TRUE(m.start(2 * sec));
  EXPECT_EQ(m.referee().robots().at("b").current_hp, 20);
  EXPECT_DOUBLE_EQ(m.referee().robots().at("r").heat, 10.0);
}
TEST(Match, GraceTransferAndFractionCarry)
{
  r::MatchEngine m(robots()); run(m);
  m.process(frame(1, {{"r", 0, 0}}));
  m.process(frame(sec / 2 + 1, {{"b", 0, 0}}));
  m.process(frame(2 * sec + 1, {{"b", 0, 0}}));
  EXPECT_EQ(m.owner(), 0);
  m.process(frame(3 * sec + 1, {{"b", 0, 0}}));
  EXPECT_EQ(m.owner(), 1); EXPECT_EQ(m.points()[1], 198); EXPECT_EQ(m.points()[0], 200);
  m.process(frame(4 * sec + 1, {{"b", 0, 0}}));
  EXPECT_EQ(m.points()[0], 199);
}
TEST(Match, ReentryCancelsExpiry)
{
  r::MatchEngine m(robots()); run(m);
  m.process(frame(1, {{"r", 0, 0}})); m.process(frame(sec));
  m.process(frame(2 * sec, {{"r", 0, 0}})); m.process(frame(4 * sec, {{"r", 0, 0}}));
  EXPECT_EQ(m.owner(), 0); EXPECT_EQ(m.eligible().size(), 1u);
}
TEST(Match, SimultaneousArrivalNeutralUntilOneSideLeaves)
{
  r::MatchEngine m(robots()); run(m);
  m.process(frame(1, {{"b", 0, 0}, {"r", 0, 0}}));
  m.process(frame(sec, {{"r", 0, 0}})); EXPECT_EQ(m.owner(), -1);
  m.process(frame(3 * sec, {{"r", 0, 0}})); EXPECT_EQ(m.owner(), 0);
  EXPECT_EQ(m.points()[1], 200);
}
TEST(Match, DeathPenaltyDamageAndInvalidHits)
{
  r::MatchEngine m(robots(7)); run(m);
  m.process(frame(1, {{"b", 0, 0}}));
  auto f = frame(sec); f.hits = {hit("r", "r2", 1, sec), hit("b", "b", 2, sec), hit("r", "b", 3, sec)};
  m.process(f);
  EXPECT_EQ(m.points()[1], 180); EXPECT_EQ(m.damage()[0], 7u);
  EXPECT_EQ(m.owner(), -1); EXPECT_EQ(m.hp()[1], 0);
  f.stamp += sec; m.process(f); EXPECT_EQ(m.points()[1], 180);
  EXPECT_EQ(m.state(), r::MatchEngine::RUNNING);  // No annihilation win.
}
TEST(Match, SimultaneousZeroAndFreeze)
{
  std::vector<r::RobotConfig> configs;
  for (int i = 0; i < 10; ++i) {
    configs.push_back({"r" + std::to_string(i), "red", 1, 88, 24});
    configs.push_back({"b" + std::to_string(i), "blue", 1, 88, 24});
  }
  r::MatchEngine m(configs); run(m);
  auto f = frame(sec);
  for (int i = 0; i < 10; ++i) {
    f.hits.push_back(hit("r0", "b" + std::to_string(i), i, sec));
    f.hits.push_back(hit("b0", "r" + std::to_string(i), i, sec));
  }
  m.process(f); EXPECT_EQ(m.result(), r::MatchEngine::DRAW);
  EXPECT_EQ(m.points()[0], 0); EXPECT_EQ(m.points()[1], 0);
  m.paused(); EXPECT_EQ(m.state(), r::MatchEngine::FINISHED);
  f.stamp += sec; m.process(f); EXPECT_EQ(m.elapsed(), 1);
}
TEST(Match, OccupationVictory)
{
  r::MatchEngine m(robots()); run(m);
  m.process(frame(1, {{"r", 0, 0}})); m.process(frame(200 * sec + 1, {{"r", 0, 0}}));
  EXPECT_EQ(m.result(), r::MatchEngine::RED_WIN); EXPECT_EQ(m.points()[1], 0);
}
TEST(Match, TimeoutTieBreakersAndCutoff)
{
  {
    r::MatchEngine m(robots()); run(m);
    auto f = frame(301 * sec); f.hits = {hit("r", "b", 1, 301 * sec)};
    m.process(f); EXPECT_EQ(m.elapsed(), 300); EXPECT_EQ(m.damage()[0], 0u);
    EXPECT_EQ(m.result(), r::MatchEngine::RED_WIN); // Two reds: more total HP.
  }
  {
    r::MatchEngine m({{"r", "red", 40, 88, 24}, {"b", "blue", 40, 88, 24}});
    run(m); m.process(frame(300 * sec));
    EXPECT_EQ(m.result(), r::MatchEngine::DRAW);
  }
  {
    r::MatchEngine m(robots()); run(m);
    auto f = frame(sec); f.hits = {hit("b", "r", 1, sec)}; m.process(f);
    m.process(frame(300 * sec)); EXPECT_EQ(m.result(), r::MatchEngine::BLUE_WIN);
  }
  {
    r::MatchEngine m(robots()); run(m);
    m.process(frame(1, {{"b", 0, 0}})); m.process(frame(sec + 1));
    m.process(frame(300 * sec)); EXPECT_EQ(m.result(), r::MatchEngine::BLUE_WIN);
  }
}
TEST(Match, AbortErrorsDisabledZoneAndCooling)
{
  r::MatchEngine m(robots(), {false, -1.5, 1.5, -1.5, 1.5});
  run(m);
  auto f = frame(1, {{"r", 0, 0}}); f.shots = {{"r", 1, "17mm", 1}}; m.process(f);
  m.process(frame(sec / 10));
  EXPECT_DOUBLE_EQ(m.referee().robots().at("r").heat, 7.6);
  m.process(frame(sec / 10)); EXPECT_DOUBLE_EQ(m.referee().robots().at("r").heat, 7.6);
  EXPECT_EQ(m.owner(), -1);
  m.end(); EXPECT_FALSE(m.start(2)); m.paused(); m.end();
  EXPECT_EQ(m.result(), r::MatchEngine::ABORTED);
  ASSERT_TRUE(m.reset(2)); m.end(); m.paused(); EXPECT_EQ(m.state(), r::MatchEngine::FINISHED);
  ASSERT_TRUE(m.reset(3)); m.fail("timeout"); EXPECT_EQ(m.state(), r::MatchEngine::ERROR);
  ASSERT_TRUE(m.reset(4)); m.reset_complete(); ASSERT_TRUE(m.start(sec)); EXPECT_TRUE(m.error().empty());
}
