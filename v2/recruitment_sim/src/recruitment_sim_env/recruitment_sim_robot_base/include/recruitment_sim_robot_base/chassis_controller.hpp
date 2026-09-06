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

#ifndef RECRUITMENT_SIM_ROBOT_BASE__CHASSIS_CONTROLLER_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__CHASSIS_CONTROLLER_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace recruitment_sim_robot_base
{

class ChassisController
{
public:
  ChassisController(
    rclcpp::Node::SharedPtr node,
    Actuator<geometry_msgs::msg::Twist>::SharedPtr chassis_actuator);
  ~ChassisController() {}

private:
  void cmd_chassis_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_chassis_vel_sub_;
  Actuator<geometry_msgs::msg::Twist>::SharedPtr chassis_actuator_;
};


}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__CHASSIS_CONTROLLER_HPP_
