// Copyright 2021 RoboMaster-OSS
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

#ifndef RECRUITMENT_SIM_ROBOT_BASE__SHOOTER_CONTROLLER_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__SHOOTER_CONTROLLER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "hardware_interface.hpp"

namespace recruitment_sim_robot_base
{

class ShooterController
{
public:
  ShooterController(
    rclcpp::Node::SharedPtr node,
    Actuator<bool>::SharedPtr shoot_actuator);
  ~ShooterController() {}

private:
  void shoot_cb(const std_msgs::msg::Bool::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub and sub
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_cmd_sub_;
  Actuator<bool>::SharedPtr shoot_actuator_;
};
}  // namespace recruitment_sim_robot_base
#endif  // RECRUITMENT_SIM_ROBOT_BASE__SHOOTER_CONTROLLER_HPP_
