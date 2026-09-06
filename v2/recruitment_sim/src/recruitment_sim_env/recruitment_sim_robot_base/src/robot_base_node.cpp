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

#include "recruitment_sim_robot_base/robot_base_node.hpp"

#include <thread>
#include <memory>
#include <string>

namespace recruitment_sim_robot_base
{

RobotBaseNode::RobotBaseNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("robot_base", options);
  gz_node_ = std::make_shared<ignition::transport::Node>();
  // parameters
  std::string world_name, robot_name;
  bool use_odometry = false;
  node_->declare_parameter("world_name", "default");
  node_->declare_parameter("robot_name", "robot");
  node_->declare_parameter("use_odometry", use_odometry);
  node_->get_parameter("robot_name", robot_name);
  node_->get_parameter("world_name", world_name);
  node_->get_parameter("use_odometry", use_odometry);
  // ign topic string
  std::string gz_chassis_cmd_topic = "/" + robot_name + "/cmd_vel";
  std::string gz_pitch_cmd_topic = "/model/" + robot_name + "/joint/gimbal_pitch_joint/cmd_vel";
  std::string gz_yaw_cmd_topic = "/model/" + robot_name + "/joint/gimbal_yaw_joint/cmd_vel";
  std::string gz_joint_state_topic = "/world/" + world_name + "/model/" + robot_name +
    "/joint_state";
  std::string gz_gimbal_imu_topic = "/world/" + world_name + "/model/" + robot_name +
    "/link/gimbal_pitch/sensor/gimbal_imu/imu";
  std::string gz_light_bar_cmd_topic = "/" + robot_name + "/color/set_state";
  // create hardware moudule
  // Actuator
  chassis_actuator_ = std::make_shared<recruitment_sim_robot_base::IgnChassisActuator>(
    node_, gz_node_, gz_chassis_cmd_topic);
  gimbal_vel_actuator_ = std::make_shared<recruitment_sim_robot_base::IgnGimbalActuator>(
    node_, gz_node_, gz_pitch_cmd_topic, gz_yaw_cmd_topic);
  shoot_actuator_ = std::make_shared<recruitment_sim_robot_base::IgnShootActuator>(
    node_, gz_node_, robot_name, "small_shooter");
  gz_light_bar_cmd_ = std::make_shared<recruitment_sim_robot_base::IgnLightBarCmd>(
    gz_node_, gz_light_bar_cmd_topic);
  // sensor wrapper
  gz_gimbal_encoder_ = std::make_shared<recruitment_sim_robot_base::IgnGimbalEncoder>(
    node_, gz_node_, gz_joint_state_topic);
  gz_gimbal_imu_ = std::make_shared<recruitment_sim_robot_base::IgnGimbalImu>(
    node_, gz_node_, gz_gimbal_imu_topic);
  // create controller and publisher
  chassis_controller_ = std::make_shared<recruitment_sim_robot_base::ChassisController>(
    node_, chassis_actuator_, gz_gimbal_encoder_->get_position_sensor());
  gimbal_controller_ = std::make_shared<recruitment_sim_robot_base::GimbalController>(
    node_, gimbal_vel_actuator_, gz_gimbal_imu_->get_position_sensor());
  shooter_controller_ = std::make_shared<recruitment_sim_robot_base::ShooterController>(
    node_, shoot_actuator_, "small_shooter_controller");
  // odometry
  if (use_odometry) {
    gz_chassis_odometry_ = std::make_shared<recruitment_sim_robot_base::IgnOdometry>(
      node_, gz_node_, "/" + robot_name + "/odometry");
    odometry_publisher_ = std::make_shared<recruitment_sim_robot_base::OdometryPublisher>(
      node_, gz_chassis_odometry_->get_odometry_sensor());
  }
  light_color_service_ = node_->create_service<
    recruitment_sim_interfaces::srv::SetLightColor>(
    "robot_base/set_light_color",
    std::bind(
      &RobotBaseNode::set_light_color_cb, this,
      std::placeholders::_1, std::placeholders::_2));
  // enable actuator and sensor
  chassis_actuator_->enable(true);
  gimbal_vel_actuator_->enable(true);
  shoot_actuator_->enable(true);
  gz_gimbal_encoder_->enable(true);
  gz_gimbal_imu_->enable(true);
  if (use_odometry) {
    gz_chassis_odometry_->enable(true);
  }
}

void RobotBaseNode::set_light_color_cb(
  const std::shared_ptr<recruitment_sim_interfaces::srv::SetLightColor::Request> request,
  std::shared_ptr<recruitment_sim_interfaces::srv::SetLightColor::Response> response)
{
  if (request->color > recruitment_sim_interfaces::srv::SetLightColor::Request::WHITE) {
    response->success = false;
    response->message = "color must be in the range [0, 4]";
    return;
  }
  gz_light_bar_cmd_->set_state(request->color);
  response->success = true;
  response->message = "light color updated";
}

}  // namespace recruitment_sim_robot_base

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(recruitment_sim_robot_base::RobotBaseNode)
