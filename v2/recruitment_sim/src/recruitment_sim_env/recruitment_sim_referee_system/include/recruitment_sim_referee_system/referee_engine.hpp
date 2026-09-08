#ifndef RECRUITMENT_SIM_REFEREE_SYSTEM__REFEREE_ENGINE_HPP_
#define RECRUITMENT_SIM_REFEREE_SYSTEM__REFEREE_ENGINE_HPP_

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace recruitment_sim_referee_system
{

struct RobotConfig
{
  std::string name;
  std::string team;
  int max_hp;
  double heat_limit;
  double cooling_rate;
};

struct RobotState
{
  RobotConfig config;
  int current_hp;
  double heat{0.0};
  std::uint32_t shots_this_period{0};
  std::uint32_t shots_last_period{0};
  std::uint64_t total_shots{0};
  std::uint64_t total_hits{0};
  bool alive{true};
  bool overheated{false};
  bool permanently_locked{false};
};

struct ShotEvent
{
  std::string shooter;
  std::uint64_t projectile_id;
  std::string projectile_type;
  std::int64_t stamp_ns;
};

struct HitEvent
{
  std::string shooter;
  std::uint64_t projectile_id;
  std::string target;
  std::string target_link;
  std::string target_collision;
  std::int64_t stamp_ns;
};

struct ControlCommand
{
  enum Target : std::uint8_t {ALL = 0, CHASSIS = 1, GIMBAL = 2, SHOOTER = 3};
  std::string robot_name;
  Target target;
  bool enabled;
};

class RefereeEngine
{
public:
  explicit RefereeEngine(const std::vector<RobotConfig> & configs);

  bool process_shot(const ShotEvent & event);
  bool process_hit(const HitEvent & event);
  void cool_one_period();

  const std::map<std::string, RobotState> & robots() const {return robots_;}
  std::vector<ControlCommand> take_control_commands();

private:
  void update_heat_lock(RobotState & robot);

  std::map<std::string, RobotState> robots_;
  std::map<std::string, std::int64_t> armor_last_hit_ns_;
  std::set<std::string> processed_hit_projectiles_;
  std::vector<ControlCommand> control_commands_;
};

}  // namespace recruitment_sim_referee_system

#endif  // RECRUITMENT_SIM_REFEREE_SYSTEM__REFEREE_ENGINE_HPP_
