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

#include "recruitment_sim_robot_base/shooter_controller.hpp"

#include <memory>

namespace recruitment_sim_robot_base
{

ShooterController::ShooterController(
  rclcpp::Node::SharedPtr node,
  Actuator<bool>::SharedPtr shoot_actuator)
: node_(node), shoot_actuator_(shoot_actuator)
{
  using namespace std::placeholders;
  shoot_cmd_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "cmd_shoot", 10, std::bind(&ShooterController::shoot_cb, this, _1));
}

void ShooterController::shoot_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  shoot_actuator_->set(msg->data);
}

}  // namespace recruitment_sim_robot_base
