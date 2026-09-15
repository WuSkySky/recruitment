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
  std::uint64_t attack_damage{0};
  bool alive{true};
  bool overheated{false};
  bool permanently_locked{false};
  // 3.3.2.2 复活机制：战亡后读条复活，复活后无敌 30 s 且进入"虚弱"。
  bool invincible{false};
  bool weakened{false};
  std::uint32_t death_count{0};
  std::int64_t revive_ready_ns{0};       ///< 读条完成时刻（仿真 ns），0 表示无待复活读条
  std::int64_t invincible_until_ns{0};   ///< 无敌结束时刻（仿真 ns）
  bool shooter_cmd{true};                ///< 上次下发的 SHOOTER 部件开关，用于去重下发
  double heal_carry{0.0};                ///< 3.3.2.1 回血的小数进位
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
  /// 结算复活读条与无敌到期。now_ns 为当前仿真时刻。
  void advance_time(std::int64_t now_ns);
  /// 结算己方补给区：解除"虚弱"与无敌，并按每秒 25% 上限血量回血。
  void supply_tick(const std::set<std::string> & in_own_supply, std::int64_t dt_ns);

  const std::map<std::string, RobotState> & robots() const {return robots_;}
  std::vector<ControlCommand> take_control_commands();

private:
  void update_heat_lock(RobotState & robot);
  /// 按"过热 / 永久锁 / 虚弱 / 存活"组合出 SHOOTER 部件开关，变化时才下发。
  void sync_shooter(RobotState & robot);

  std::map<std::string, RobotState> robots_;
  std::map<std::string, std::int64_t> armor_last_hit_ns_;
  std::set<std::string> processed_hit_projectiles_;
  std::vector<ControlCommand> control_commands_;
};

}  // namespace recruitment_sim_referee_system

#endif  // RECRUITMENT_SIM_REFEREE_SYSTEM__REFEREE_ENGINE_HPP_
