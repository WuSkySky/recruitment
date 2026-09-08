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

#ifndef RECRUITMENT_SIM_ROBOT_BASE__GIMBAL_INTERFACE_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__GIMBAL_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "ignition/msgs/double.pb.h"
#include "ignition/msgs/model.pb.h"
#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace recruitment_sim_robot_base
{

class GimbalInterface
{
public:
  GimbalInterface(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_pitch_cmd_topic,
    const std::string & gz_yaw_cmd_topic,
    const std::string & gz_joint_state_topic);
  void enable(bool enabled);

private:
  void yaw_velocity_cb(const std_msgs::msg::Float64::SharedPtr msg);
  void pitch_velocity_cb(const std_msgs::msg::Float64::SharedPtr msg);
  void gz_joint_state_cb(const ignition::msgs::Model & msg);
  void publish_commands();
  void publish_feedback();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_pitch_cmd_pub_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_yaw_cmd_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_velocity_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_velocity_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_velocity_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_angle_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_angle_feedback_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  std::atomic<double> yaw_velocity_command_{0.0};
  std::atomic<double> pitch_velocity_command_{0.0};
  std::mutex feedback_mutex_;
  double yaw_position_{0.0};
  double pitch_position_{0.0};
  double yaw_velocity_{0.0};
  double pitch_velocity_{0.0};
  bool has_yaw_feedback_{false};
  bool has_pitch_feedback_{false};
  std::atomic<bool> enabled_{false};
};

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__GIMBAL_INTERFACE_HPP_
