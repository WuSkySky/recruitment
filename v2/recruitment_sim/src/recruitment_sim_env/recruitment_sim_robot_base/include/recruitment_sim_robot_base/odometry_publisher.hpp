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

#ifndef RECRUITMENT_SIM_ROBOT_BASE__ODOMETRY_PUBLISHER_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__ODOMETRY_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "hardware_interface.hpp"
#include "recruitment_sim_robot_base/gaussian_noise.hpp"
#include "recruitment_sim_robot_base/incremental_odometry.hpp"
#include "recruitment_sim_robot_base/initialization_gate.hpp"

namespace recruitment_sim_robot_base
{

class OdometryPublisher
{
public:
  OdometryPublisher(
    rclcpp::Node::SharedPtr node,
    Sensor<nav_msgs::msg::Odometry>::SharedPtr odometry_sensor,
    double x_position_increment_variance,
    double y_position_increment_variance,
    double yaw_increment_variance,
    const std::string & publisher_name = "odometry_publisher");
  ~OdometryPublisher() {}
  bool request_initialize(uint64_t round_id);

private:
  void timer_callback();

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // sensor data
  std::mutex msg_mut_;
  std::mutex state_mut_;
  Sensor<nav_msgs::msg::Odometry>::SharedPtr odometry_sensor_;
  nav_msgs::msg::Odometry sensor_msg_;
  bool has_sensor_msg_{false};
  uint64_t sensor_sequence_{0};
  bool odometry_initialized_{false};
  InitializationGate initialization_gate_;
  PlanarPose last_ground_truth_;
  PlanarPose last_odometry_;
  GaussianNoise x_position_increment_noise_;
  GaussianNoise y_position_increment_noise_;
  GaussianNoise yaw_increment_noise_;
  std::string frame_id_{"odom"};
  std::string child_frame_id_{"base_link"};
  bool use_footprint_{false};
};
}  // namespace recruitment_sim_robot_base
#endif  // RECRUITMENT_SIM_ROBOT_BASE__ODOMETRY_PUBLISHER_HPP_
