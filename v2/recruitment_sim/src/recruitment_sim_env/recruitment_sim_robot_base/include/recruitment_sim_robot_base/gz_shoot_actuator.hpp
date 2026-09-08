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

#ifndef RECRUITMENT_SIM_ROBOT_BASE__GZ_SHOOT_ACTUATOR_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__GZ_SHOOT_ACTUATOR_HPP_

#include <memory>
#include <string>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"

namespace recruitment_sim_robot_base
{

class IgnShootActuator : public Actuator<bool>
{
public:
  IgnShootActuator(
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & robot_name,
    const std::string & shooter_name);
  ~IgnShootActuator() {}

  void set(const bool & enabled) override;
  void enable(bool enable);

private:
  void publish(bool enabled);

private:
  std::shared_ptr<ignition::transport::Node> gz_node_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_shoot_cmd_pub_;
  bool enable_{false};
};

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__GZ_SHOOT_ACTUATOR_HPP_
