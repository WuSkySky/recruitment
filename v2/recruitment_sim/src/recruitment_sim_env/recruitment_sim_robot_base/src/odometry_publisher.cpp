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

#include "recruitment_sim_robot_base/odometry_publisher.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace recruitment_sim_robot_base
{

OdometryPublisher::OdometryPublisher(
  rclcpp::Node::SharedPtr node,
  Sensor<nav_msgs::msg::Odometry>::SharedPtr odometry_sensor,
  double x_position_increment_variance,
  double y_position_increment_variance,
  double yaw_increment_variance,
  const std::string & publisher_name)
: node_(node), odometry_sensor_(odometry_sensor),
  x_position_increment_noise_(x_position_increment_variance),
  y_position_increment_noise_(y_position_increment_variance),
  yaw_increment_noise_(yaw_increment_variance)
{
  odometry_sensor_->add_callback(
    [this](const nav_msgs::msg::Odometry & data, const rclcpp::Time & stamp) {
      std::lock_guard<std::mutex> lock(msg_mut_);
      sensor_msg_ = data;
      sensor_msg_.header.stamp = stamp;
      has_sensor_msg_ = true;
      ++sensor_sequence_;
    });
  // parameters
  int rate = 30;
  std::string param_ns = publisher_name + ".";
  node_->declare_parameter(param_ns + "rate", rate);
  node_->declare_parameter(param_ns + "frame_id", frame_id_);
  node_->declare_parameter(param_ns + "child_frame_id", child_frame_id_);
  node_->declare_parameter(param_ns + "use_footprint", use_footprint_);
  node_->get_parameter(param_ns + "rate", rate);
  node_->get_parameter(param_ns + "frame_id", frame_id_);
  node_->get_parameter(param_ns + "child_frame_id", child_frame_id_);
  node_->get_parameter(param_ns + "use_footprint", use_footprint_);
  // create ros pub and timer
  std::string odom_topic = "robot_base/odom";
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  auto period = std::chrono::microseconds(1000000 / rate);
  timer_ = node_->create_wall_timer(
    period, std::bind(&OdometryPublisher::timer_callback, this));
}

bool OdometryPublisher::request_initialize(uint64_t round_id)
{
  std::unique_lock<std::mutex> msg_lock(msg_mut_, std::defer_lock);
  std::unique_lock<std::mutex> state_lock(state_mut_, std::defer_lock);
  std::lock(msg_lock, state_lock);
  return initialization_gate_.request(round_id, sensor_sequence_) !=
         InitializationRequestResult::STALE;
}

void OdometryPublisher::timer_callback()
{
  nav_msgs::msg::Odometry ground_truth;
  uint64_t sensor_sequence;
  {
    std::lock_guard<std::mutex> lock(msg_mut_);
    if (!has_sensor_msg_) {
      return;
    }
    ground_truth = sensor_msg_;
    sensor_sequence = sensor_sequence_;
  }

  std::lock_guard<std::mutex> state_lock(state_mut_);
  if (initialization_gate_.blocks(sensor_sequence)) {
    return;
  }
  const bool initialize_now = initialization_gate_.consume_if_ready(sensor_sequence);

  const auto & orientation = ground_truth.pose.pose.orientation;
  const double ground_truth_yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  const PlanarPose current_ground_truth{
    ground_truth.pose.pose.position.x,
    ground_truth.pose.pose.position.y,
    ground_truth_yaw};

  if (!odometry_initialized_ || initialize_now) {
    last_ground_truth_ = current_ground_truth;
    last_odometry_ = PlanarPose{};
    odometry_initialized_ = true;
  } else {
    last_odometry_ = integrate_odometry_increment(
      last_ground_truth_, current_ground_truth, last_odometry_,
      x_position_increment_noise_.apply(0.0),
      y_position_increment_noise_.apply(0.0),
      yaw_increment_noise_.apply(0.0));
    last_ground_truth_ = current_ground_truth;
  }

  nav_msgs::msg::Odometry odom_msg = ground_truth;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.child_frame_id = child_frame_id_;
  odom_msg.pose.pose.position.x = last_odometry_.x;
  odom_msg.pose.pose.position.y = last_odometry_.y;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = std::sin(last_odometry_.yaw * 0.5);
  odom_msg.pose.pose.orientation.w = std::cos(last_odometry_.yaw * 0.5);
  if (use_footprint_) {
    odom_msg.pose.pose.position.z = 0;
  }
  odom_pub_->publish(odom_msg);
}

}  // namespace recruitment_sim_robot_base
