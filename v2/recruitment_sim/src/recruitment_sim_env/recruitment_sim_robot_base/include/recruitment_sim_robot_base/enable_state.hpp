#ifndef RECRUITMENT_SIM_ROBOT_BASE__ENABLE_STATE_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__ENABLE_STATE_HPP_

#include <cstdint>

namespace recruitment_sim_robot_base
{

class EnableState
{
public:
  enum Target : std::uint8_t {ALL = 0, CHASSIS = 1, GIMBAL = 2, SHOOTER = 3};

  bool set(std::uint8_t target, bool enabled)
  {
    switch (target) {
      case ALL:
        all_enabled_ = enabled;
        return true;
      case CHASSIS:
        chassis_enabled_ = enabled;
        return true;
      case GIMBAL:
        gimbal_enabled_ = enabled;
        return true;
      case SHOOTER:
        shooter_enabled_ = enabled;
        return true;
      default:
        return false;
    }
  }

  bool chassis_enabled() const {return all_enabled_ && chassis_enabled_;}
  bool gimbal_enabled() const {return all_enabled_ && gimbal_enabled_;}
  bool shooter_enabled() const {return all_enabled_ && shooter_enabled_;}
  void reset(bool enabled = true)
  {
    all_enabled_ = enabled;
    chassis_enabled_ = true;
    gimbal_enabled_ = true;
    shooter_enabled_ = true;
  }

private:
  bool all_enabled_{true};
  bool chassis_enabled_{true};
  bool gimbal_enabled_{true};
  bool shooter_enabled_{true};
};

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__ENABLE_STATE_HPP_
