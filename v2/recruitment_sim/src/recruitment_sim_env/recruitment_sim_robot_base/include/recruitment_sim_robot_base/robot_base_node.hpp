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

#ifndef RECRUITMENT_SIM_ROBOT_BASE__ROBOT_BASE_NODE_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__ROBOT_BASE_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "recruitment_sim_robot_base/gz_chassis_actuator.hpp"
#include "recruitment_sim_robot_base/gz_shoot_actuator.hpp"
#include "recruitment_sim_robot_base/gz_odometry.hpp"
#include "recruitment_sim_robot_base/gz_light_bar_cmd.hpp"
#include "recruitment_sim_robot_base/enable_state.hpp"

#include "recruitment_sim_robot_base/chassis_controller.hpp"
#include "recruitment_sim_robot_base/gimbal_interface.hpp"
#include "recruitment_sim_robot_base/shooter_controller.hpp"
#include "recruitment_sim_robot_base/odometry_publisher.hpp"
#include "recruitment_sim_interfaces/srv/set_light_color.hpp"
#include "recruitment_sim_interfaces/srv/set_robot_enabled.hpp"

namespace recruitment_sim_robot_base
{
// Node wrapper for RobotBaseNode
class RobotBaseNode
{
public:
  explicit RobotBaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

  void set_light_color_cb(
    const std::shared_ptr<recruitment_sim_interfaces::srv::SetLightColor::Request> request,
    std::shared_ptr<recruitment_sim_interfaces::srv::SetLightColor::Response> response);
  void set_robot_enabled_cb(
    const std::shared_ptr<recruitment_sim_interfaces::srv::SetRobotEnabled::Request> request,
    std::shared_ptr<recruitment_sim_interfaces::srv::SetRobotEnabled::Response> response);

private:
  void apply_enabled_state();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  rclcpp::Service<recruitment_sim_interfaces::srv::SetLightColor>::SharedPtr light_color_service_;
  rclcpp::Service<recruitment_sim_interfaces::srv::SetRobotEnabled>::SharedPtr enabled_service_;
  // ign actuator moudule
  std::shared_ptr<recruitment_sim_robot_base::IgnChassisActuator> chassis_actuator_;
  std::shared_ptr<recruitment_sim_robot_base::IgnShootActuator> shoot_actuator_;
  std::shared_ptr<recruitment_sim_robot_base::IgnLightBarCmd> gz_light_bar_cmd_;
  // ign sensor moudule
  std::shared_ptr<recruitment_sim_robot_base::IgnOdometry> gz_chassis_odometry_;
  // ros controller/publisher wrapper
  std::shared_ptr<recruitment_sim_robot_base::ChassisController> chassis_controller_;
  std::shared_ptr<recruitment_sim_robot_base::GimbalInterface> gimbal_interface_;
  std::shared_ptr<recruitment_sim_robot_base::ShooterController> shooter_controller_;
  std::shared_ptr<recruitment_sim_robot_base::OdometryPublisher> odometry_publisher_;
  EnableState enable_state_;
};

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__ROBOT_BASE_NODE_HPP_
