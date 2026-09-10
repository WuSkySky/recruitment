#ifndef RECRUITMENT_SIM_ROBOT_BASE__INITIALIZATION_GATE_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__INITIALIZATION_GATE_HPP_

#include <cstdint>

namespace recruitment_sim_robot_base
{

enum class InitializationRequestResult {SCHEDULED, IDEMPOTENT, STALE};

class InitializationGate
{
public:
  InitializationRequestResult request(uint64_t round_id, uint64_t current_sequence)
  {
    if (has_round_ && round_id < round_id_) {
      return InitializationRequestResult::STALE;
    }
    if (has_round_ && round_id == round_id_) {
      return InitializationRequestResult::IDEMPOTENT;
    }
    has_round_ = true;
    round_id_ = round_id;
    pending_ = true;
    after_sequence_ = current_sequence;
    return InitializationRequestResult::SCHEDULED;
  }

  bool blocks(uint64_t sequence) const
  {
    return pending_ && sequence <= after_sequence_;
  }

  bool consume_if_ready(uint64_t sequence)
  {
    if (!pending_ || sequence <= after_sequence_) {
      return false;
    }
    pending_ = false;
    return true;
  }

private:
  bool has_round_{false};
  bool pending_{false};
  uint64_t round_id_{0};
  uint64_t after_sequence_{0};
};

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__INITIALIZATION_GATE_HPP_
