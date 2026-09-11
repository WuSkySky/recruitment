// Copyright 2026 Recruitment Simulation Maintainers
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <recruitment_sim_interfaces/msg/simulation_event.hpp>
#include <map>
#include <mutex>
#include <thread>
#include <functional>
namespace recruitment_sim {
template<class T> struct Signature;
template<class C, class R, class... A> struct Signature<R(C::*)(A...) const> {using type=std::function<R(A...)>;};
using Event = recruitment_sim_interfaces::msg::SimulationEvent;
// One instance shared by all loaded plugin libraries in a server.
struct RobotHandle {
  gazebo::physics::ModelPtr model;
  ignition::math::Pose3d spawn;
  std::function<void()> reset;
  std::function<void()> contacts;
};
struct SharedState {
  std::recursive_mutex mutex;
  uint64_t round{0};
  std::vector<Event> events;
  std::map<std::string, RobotHandle> robots;
  std::vector<std::string> projectiles;
};
SharedState & State();
class Domain {
public:
  explicit Domain(size_t id);
  ~Domain();
  rclcpp::Node::SharedPtr Node(const std::string & name, const std::string & ns);
  void Remove(const rclcpp::Node::SharedPtr & node);
private:
  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread worker_;
};
std::shared_ptr<Domain> GetDomain(size_t id);
size_t InternalDomain();
inline int64_t Nanoseconds(const gazebo::common::Time & t) {
  return static_cast<int64_t>(t.sec)*1000000000LL+t.nsec;
}
}
