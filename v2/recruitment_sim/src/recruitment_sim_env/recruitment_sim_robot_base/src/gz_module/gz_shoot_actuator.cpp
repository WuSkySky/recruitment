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
#include "recruitment_sim_robot_base/gz_shoot_actuator.hpp"

#include "ignition/msgs/boolean.pb.h"

#include <memory>
#include <string>

namespace recruitment_sim_robot_base
{

IgnShootActuator::IgnShootActuator(
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & robot_name,
  const std::string & shooter_name)
: gz_node_(gz_node)
{
  std::string gz_shoot_cmd_topic = "/" + robot_name + "/" + shooter_name + "/shoot";
  gz_shoot_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Boolean>(gz_shoot_cmd_topic));
}

void IgnShootActuator::set(const bool & enabled)
{
  if (!enable_) {
    return;
  }
  ignition::msgs::Boolean gz_msg;
  gz_msg.set_data(enabled);
  gz_shoot_cmd_pub_->Publish(gz_msg);
}

}  // namespace recruitment_sim_robot_base
