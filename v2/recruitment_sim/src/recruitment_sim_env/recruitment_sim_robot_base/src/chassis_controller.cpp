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
#include "recruitment_sim_robot_base/chassis_controller.hpp"

#include <memory>

namespace recruitment_sim_robot_base
{
ChassisController::ChassisController(
  rclcpp::Node::SharedPtr node,
  Actuator<geometry_msgs::msg::Twist>::SharedPtr chassis_actuator)
: node_(node), chassis_actuator_(chassis_actuator)
{
  using namespace std::placeholders;
  cmd_chassis_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_chassis_vel", 10, std::bind(&ChassisController::cmd_chassis_vel_cb, this, _1));
}

void ChassisController::cmd_chassis_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  chassis_actuator_->set(*msg);
}

}  // namespace recruitment_sim_robot_base
