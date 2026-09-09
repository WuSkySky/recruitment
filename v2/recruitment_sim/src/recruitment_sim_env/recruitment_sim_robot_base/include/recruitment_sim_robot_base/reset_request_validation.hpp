// Copyright 2026 Recruitment Simulation Maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RECRUITMENT_SIM_ROBOT_BASE__RESET_REQUEST_VALIDATION_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__RESET_REQUEST_VALIDATION_HPP_

#include <cstdint>

#include "recruitment_sim_interfaces/srv/reset_robot.hpp"

namespace recruitment_sim_robot_base
{

enum class ResetRequestError
{
  NONE,
  STALE_ROUND,
  INVALID_COLOR,
};

inline ResetRequestError validate_reset_request(
  uint64_t current_round, uint64_t requested_round, uint8_t color)
{
  if (requested_round < current_round) {
    return ResetRequestError::STALE_ROUND;
  }
  if (color > recruitment_sim_interfaces::srv::ResetRobot::Request::WHITE) {
    return ResetRequestError::INVALID_COLOR;
  }
  return ResetRequestError::NONE;
}

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__RESET_REQUEST_VALIDATION_HPP_
