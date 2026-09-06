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

#include "recruitment_sim_robot_base/gimbal_interface.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

namespace recruitment_sim_robot_base
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;

double normalize_angle(double angle)
{
  return std::remainder(angle, kTwoPi);
}
}  // namespace

GimbalInterface::GimbalInterface(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & gz_pitch_cmd_topic,
  const std::string & gz_yaw_cmd_topic,
  const std::string & gz_joint_state_topic)
: node_(std::move(node)), gz_node_(std::move(gz_node))
{
  gz_pitch_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Double>(gz_pitch_cmd_topic));
  gz_yaw_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Double>(gz_yaw_cmd_topic));
  gz_node_->Subscribe(gz_joint_state_topic, &GimbalInterface::gz_joint_state_cb, this);

  using namespace std::placeholders;
  yaw_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "cmd_yaw_vel", 10, std::bind(&GimbalInterface::yaw_velocity_cb, this, _1));
  pitch_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "cmd_pitch_vel", 10, std::bind(&GimbalInterface::pitch_velocity_cb, this, _1));

  yaw_velocity_feedback_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "feedback_yaw_vel", 10);
  pitch_velocity_feedback_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "feedback_pitch_vel", 10);
  yaw_angle_feedback_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "feedback_yaw_angle", 10);
  pitch_angle_feedback_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "feedback_pitch_angle", 10);

  // Keep both joints at zero relative velocity until a gimbal command arrives.
  // Without an initial command, Gazebo leaves the revolute joints passive, so
  // the chassis can rotate underneath the gimbal while it stays world-fixed.
  command_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&GimbalInterface::publish_commands, this));
  feedback_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&GimbalInterface::publish_feedback, this));
}

void GimbalInterface::yaw_velocity_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
  yaw_velocity_command_.store(msg->data);
}

void GimbalInterface::pitch_velocity_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
  pitch_velocity_command_.store(msg->data);
}

void GimbalInterface::publish_commands()
{
  ignition::msgs::Double gz_msg;
  gz_msg.set_data(yaw_velocity_command_.load());
  gz_yaw_cmd_pub_->Publish(gz_msg);
  gz_msg.set_data(pitch_velocity_command_.load());
  gz_pitch_cmd_pub_->Publish(gz_msg);
}

void GimbalInterface::gz_joint_state_cb(const ignition::msgs::Model & msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  for (int i = 0; i < msg.joint_size(); ++i) {
    const auto & joint = msg.joint(i);
    if (joint.name().find("gimbal_yaw_joint") != std::string::npos) {
      yaw_position_ = joint.axis1().position();
      yaw_velocity_ = joint.axis1().velocity();
      has_yaw_feedback_ = true;
    } else if (joint.name().find("gimbal_pitch_joint") != std::string::npos) {
      pitch_position_ = joint.axis1().position();
      pitch_velocity_ = joint.axis1().velocity();
      has_pitch_feedback_ = true;
    }
  }
}

void GimbalInterface::publish_feedback()
{
  double yaw_position;
  double pitch_position;
  double yaw_velocity;
  double pitch_velocity;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!has_yaw_feedback_ || !has_pitch_feedback_) {
      return;
    }
    yaw_position = yaw_position_;
    pitch_position = pitch_position_;
    yaw_velocity = yaw_velocity_;
    pitch_velocity = pitch_velocity_;
  }

  std_msgs::msg::Float64 feedback;
  feedback.data = yaw_velocity;
  yaw_velocity_feedback_pub_->publish(feedback);
  feedback.data = pitch_velocity;
  pitch_velocity_feedback_pub_->publish(feedback);
  feedback.data = normalize_angle(yaw_position);
  yaw_angle_feedback_pub_->publish(feedback);
  feedback.data = normalize_angle(pitch_position);
  pitch_angle_feedback_pub_->publish(feedback);
}

}  // namespace recruitment_sim_robot_base
