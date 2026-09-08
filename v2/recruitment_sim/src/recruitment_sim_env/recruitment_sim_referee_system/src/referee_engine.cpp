#include "recruitment_sim_referee_system/referee_engine.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace recruitment_sim_referee_system
{

namespace
{
constexpr double kHeatPer17mmProjectile = 10.0;
constexpr int kDamagePer17mmProjectile = 20;
constexpr std::int64_t kArmorDetectionIntervalNs = 50000000;

std::string projectile_key(const std::string & shooter, std::uint64_t id)
{
  return shooter + "#" + std::to_string(id);
}
}  // namespace

RefereeEngine::RefereeEngine(const std::vector<RobotConfig> & configs)
{
  for (const auto & config : configs) {
    if (config.name.empty() || config.team.empty() || config.max_hp <= 0 ||
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
    event.target_link.rfind("armor_", 0) != 0 || !target_it->second.alive)
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
  target.current_hp = std::max(0, target.current_hp - kDamagePer17mmProjectile);
  if (target.current_hp == 0 && target.alive) {
    target.alive = false;
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
      control_commands_.push_back({robot.config.name, ControlCommand::SHOOTER, true});
    }
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
    control_commands_.push_back({robot.config.name, ControlCommand::SHOOTER, false});
  }
}

}  // namespace recruitment_sim_referee_system
