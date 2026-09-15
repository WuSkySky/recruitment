#include "recruitment_sim_referee_system/referee_engine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace recruitment_sim_referee_system
{

namespace
{
constexpr double kHeatPer17mmProjectile = 10.0;
constexpr int kDamagePer17mmProjectile = 20;
constexpr std::int64_t kArmorDetectionIntervalNs = 50000000;
// 3.3.2.2 复活机制：首次战亡读条 5 点，之后每次 +5；每秒自动 +1 点。
constexpr std::int64_t kSecondNs = 1000000000LL;
constexpr std::int64_t kReviveCastBaseSeconds = 5;
constexpr std::int64_t kReviveCastStepSeconds = 5;
constexpr double kReviveHpRatio = 0.20;
constexpr std::int64_t kInvincibleSeconds = 30;
// 3.3.2.1 回血机制：占领己方补给区时每秒恢复上限血量的 25%。
constexpr double kHealRatioPerSecond = 0.25;

std::string projectile_key(const std::string & shooter, std::uint64_t id)
{
  return shooter + "#" + std::to_string(id);
}
}  // namespace

RefereeEngine::RefereeEngine(const std::vector<RobotConfig> & configs)
{
  for (const auto & config : configs) {
    if (config.name.empty() || config.team.empty() || config.max_hp <= 0 ||
      !std::isfinite(config.heat_limit) || !std::isfinite(config.cooling_rate) ||
      config.heat_limit <= 0.0 || config.cooling_rate <= 0.0)
    {
      throw std::invalid_argument("invalid robot referee configuration");
    }
    RobotState state;
    state.config = config;
    state.current_hp = config.max_hp;
    if (!robots_.emplace(config.name, std::move(state)).second) {
      throw std::invalid_argument("duplicate robot name: " + config.name);
    }
  }
  if (robots_.empty()) {
    throw std::invalid_argument("at least one robot is required");
  }
}

bool RefereeEngine::process_shot(const ShotEvent & event)
{
  auto robot_it = robots_.find(event.shooter);
  if (robot_it == robots_.end() || event.projectile_type != "17mm") {
    return false;
  }

  auto & robot = robot_it->second;
  ++robot.shots_this_period;
  ++robot.total_shots;
  robot.heat += kHeatPer17mmProjectile;
  update_heat_lock(robot);
  return true;
}

bool RefereeEngine::process_hit(const HitEvent & event)
{
  auto shooter_it = robots_.find(event.shooter);
  auto target_it = robots_.find(event.target);
  if (shooter_it == robots_.end() || target_it == robots_.end() ||
    event.shooter == event.target ||
    shooter_it->second.config.team == target_it->second.config.team ||
    event.target_collision != "target_collision" ||
    event.target_link.rfind("armor_", 0) != 0 || !target_it->second.alive ||
    target_it->second.invincible)
  {
    return false;
  }

  const auto projectile = projectile_key(event.shooter, event.projectile_id);
  if (processed_hit_projectiles_.count(projectile) != 0) {
    return false;
  }

  const std::string armor = event.target + "/" + event.target_link;
  const auto last_hit = armor_last_hit_ns_.find(armor);
  if (last_hit != armor_last_hit_ns_.end() && event.stamp_ns >= last_hit->second &&
    event.stamp_ns - last_hit->second < kArmorDetectionIntervalNs)
  {
    return false;
  }

  processed_hit_projectiles_.insert(projectile);
  armor_last_hit_ns_[armor] = event.stamp_ns;
  ++shooter_it->second.total_hits;
  auto & target = target_it->second;
  shooter_it->second.attack_damage += std::min(target.current_hp, kDamagePer17mmProjectile);
  target.current_hp = std::max(0, target.current_hp - kDamagePer17mmProjectile);
  target.heal_carry = 0.0;
  if (target.current_hp == 0 && target.alive) {
    target.alive = false;
    // 3.3.2.2：战亡时射击热量重置为 0。缓冲能量本项目未建模。
    target.heat = 0.0;
    target.overheated = false;
    target.heal_carry = 0.0;
    target.weakened = false;
    target.invincible = false;
    target.invincible_until_ns = 0;
    ++target.death_count;
    // 首次战亡读条 5 点，此后每次 +5；每秒自动 +1 点。
    target.revive_ready_ns =
      event.stamp_ns + kReviveCastBaseSeconds * target.death_count * kSecondNs;
    // permanently_locked 属于"当局锁定"，按 3.3.1.3 不因死亡重置。
    control_commands_.push_back({target.config.name, ControlCommand::ALL, false});
  }
  return true;
}

void RefereeEngine::cool_one_period()
{
  for (auto & item : robots_) {
    auto & robot = item.second;
    robot.shots_last_period = robot.shots_this_period;
    robot.shots_this_period = 0;
    robot.heat = std::max(0.0, robot.heat - robot.config.cooling_rate / 10.0);
    if (robot.overheated && !robot.permanently_locked && robot.heat <= 0.0) {
      robot.overheated = false;
    }
    sync_shooter(robot);
  }
}

void RefereeEngine::advance_time(std::int64_t now_ns)
{
  for (auto & item : robots_) {
    auto & robot = item.second;
    // 无敌固定 30 s；"虚弱"只能靠进入己方补给区解除，不随无敌一起超时。
    if (robot.invincible && now_ns >= robot.invincible_until_ns) {
      robot.invincible = false;
      robot.invincible_until_ns = 0;
    }
    if (!robot.alive && robot.revive_ready_ns > 0 && now_ns >= robot.revive_ready_ns) {
      robot.alive = true;
      robot.current_hp = std::max(
        1, static_cast<int>(std::lround(robot.config.max_hp * kReviveHpRatio)));
      robot.weakened = true;
      robot.invincible = true;
      robot.invincible_until_ns = now_ns + kInvincibleSeconds * kSecondNs;
      robot.revive_ready_ns = 0;
      robot.heal_carry = 0.0;
      control_commands_.push_back({robot.config.name, ControlCommand::ALL, true});
      sync_shooter(robot);
    }
  }
}

void RefereeEngine::supply_tick(
  const std::set<std::string> & in_own_supply, std::int64_t dt_ns)
{
  if (dt_ns <= 0) {return;}
  for (const auto & name : in_own_supply) {
    auto it = robots_.find(name);
    if (it == robots_.end() || !it->second.alive) {continue;}
    auto & robot = it->second;
    // 3.3.2.2：检测到己方补给区时"虚弱"与无敌一并解除。
    if (robot.weakened || robot.invincible) {
      robot.weakened = false;
      robot.invincible = false;
      robot.invincible_until_ns = 0;
      sync_shooter(robot);
    }
    if (robot.current_hp >= robot.config.max_hp) {robot.heal_carry = 0.0; continue;}
    // 3.3.2.1：每秒恢复上限血量的 25%，小数进位累计。
    robot.heal_carry += kHealRatioPerSecond * robot.config.max_hp *
      static_cast<double>(dt_ns) / static_cast<double>(kSecondNs);
    const int whole = static_cast<int>(robot.heal_carry);
    if (whole > 0) {
      robot.heal_carry -= whole;
      robot.current_hp = std::min(robot.config.max_hp, robot.current_hp + whole);
    }
  }
}

void RefereeEngine::sync_shooter(RobotState & robot)
{
  // 战亡期间由 ALL=false 统一断电，这里不重复下发部件级命令。
  if (!robot.alive) {return;}
  const bool want = !robot.overheated && !robot.permanently_locked && !robot.weakened;
  if (want != robot.shooter_cmd) {
    robot.shooter_cmd = want;
    control_commands_.push_back({robot.config.name, ControlCommand::SHOOTER, want});
  }
}

std::vector<ControlCommand> RefereeEngine::take_control_commands()
{
  std::vector<ControlCommand> result;
  result.swap(control_commands_);
  return result;
}

void RefereeEngine::update_heat_lock(RobotState & robot)
{
  if (robot.heat >= robot.config.heat_limit + 100.0) {
    robot.permanently_locked = true;
  }
  if ((robot.heat > robot.config.heat_limit || robot.permanently_locked) && !robot.overheated) {
    robot.overheated = true;
    sync_shooter(robot);
  }
}

}  // namespace recruitment_sim_referee_system
