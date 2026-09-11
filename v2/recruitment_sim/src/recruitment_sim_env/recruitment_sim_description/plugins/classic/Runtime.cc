// Copyright 2026 Recruitment Simulation Maintainers
// SPDX-License-Identifier: Apache-2.0
#include "Runtime.hh"
#include <cstdlib>
namespace recruitment_sim {
SharedState & State() {static SharedState state; return state;}
Domain::Domain(size_t id) {
  context_ = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init; init.set_domain_id(id); init.auto_initialize_logging(false);
  context_->init(0, nullptr, init);
  rclcpp::ExecutorOptions options; options.context = context_;
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);
  worker_ = std::thread([this] {executor_->spin();});
}
Domain::~Domain() {executor_->cancel(); if(worker_.joinable()) worker_.join(); context_->shutdown("plugin domain unloaded");}
rclcpp::Node::SharedPtr Domain::Node(const std::string & name, const std::string & ns) {
  rclcpp::NodeOptions options; options.context(context_).use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>(name, ns, options); executor_->add_node(node); return node;
}
void Domain::Remove(const rclcpp::Node::SharedPtr & node) {if(node) executor_->remove_node(node);}
std::shared_ptr<Domain> GetDomain(size_t id) {
  static std::mutex mutex;
  static std::map<size_t, std::weak_ptr<Domain>> domains;
  std::lock_guard<std::mutex> lock(mutex);
  auto domain = domains[id].lock();
  if(!domain) {domain = std::make_shared<Domain>(id); domains[id] = domain;}
  return domain;
}
size_t InternalDomain() {const char * value = std::getenv("ROS_DOMAIN_ID"); return value ? std::stoul(value) : 0;}
}
