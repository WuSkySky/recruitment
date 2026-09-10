// Small-message whitelist gateway. Large sensor streams never enter this process.
#include <memory>
#include <stdexcept>
#include <string>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <recruitment_sim_interfaces/msg/match_status.hpp>
#include <recruitment_sim_interfaces/msg/player_input.hpp>
#include <recruitment_sim_interfaces/msg/robot_status.hpp>

using Subscriptions = std::vector<rclcpp::SubscriptionBase::SharedPtr>;

template<typename Message>
void forward(const rclcpp::Node::SharedPtr & source,
  const rclcpp::Node::SharedPtr & destination, const std::string & topic,
  Subscriptions & subscriptions, const rclcpp::QoS & qos = rclcpp::QoS(10))
{
  auto publisher = destination->create_publisher<Message>(topic, qos);
  subscriptions.push_back(source->create_subscription<Message>(topic, qos,
    [publisher](const typename Message::SharedPtr message) {publisher->publish(*message);}));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  auto team_context = std::make_shared<rclcpp::Context>();
  try {
    auto internal = std::make_shared<rclcpp::Node>("team_topic_bridge");
    const auto domain = internal->declare_parameter<int>("team_domain", 20);
    const auto team = internal->declare_parameter<std::string>("team", "red");
    const auto namespaces = internal->declare_parameter<std::vector<std::string>>(
      "robot_namespaces", {"red/infantry"});
    const auto names = internal->declare_parameter<std::vector<std::string>>(
      "robot_names", {"red_infantry_robot"});
    const auto internal_domain = internal->get_node_base_interface()->get_context()->get_domain_id();
    if (domain < 0 || static_cast<size_t>(domain) == internal_domain) {
      throw std::invalid_argument("Internal and team domains must differ; domain must be nonnegative");
    }
    if (names.size() != namespaces.size()) {
      throw std::invalid_argument("robot_names and robot_namespaces must have equal lengths");
    }
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(domain);
    init_options.auto_initialize_logging(false);
    team_context->init(0, nullptr, init_options);
    rclcpp::NodeOptions node_options;
    node_options.context(team_context).use_global_arguments(false);
    auto external = std::make_shared<rclcpp::Node>(team + "_topic_gateway", node_options);
    Subscriptions subscriptions;
    for (size_t i = 0; i < names.size(); ++i) {
      const auto prefix = "/" + namespaces[i];
      if (prefix.rfind("/" + team + "/", 0) != 0) {
        throw std::invalid_argument("Robot namespace does not belong to configured team");
      }
      forward<geometry_msgs::msg::Twist>(external, internal,
        prefix + "/cmd_chassis_vel", subscriptions);
      forward<std_msgs::msg::Bool>(external, internal, prefix + "/cmd_shoot", subscriptions);
      for (const auto & suffix : {"cmd_yaw_vel", "cmd_pitch_vel"}) {
        forward<std_msgs::msg::Float64>(external, internal, prefix + "/" + suffix, subscriptions);
      }
      for (const auto & suffix : {"feedback_yaw_vel", "feedback_pitch_vel",
        "feedback_yaw_angle", "feedback_pitch_angle"})
      {
        forward<std_msgs::msg::Float64>(internal, external, prefix + "/" + suffix, subscriptions);
      }
      forward<recruitment_sim_interfaces::msg::RobotStatus>(internal, external,
        "/referee_system/" + names[i] + "/status", subscriptions);
      if (namespaces[i] == team + "/infantry") {
        forward<recruitment_sim_interfaces::msg::PlayerInput>(internal, external,
          prefix + "/player_input", subscriptions);
      }
    }
    forward<rosgraph_msgs::msg::Clock>(internal, external, "/clock", subscriptions,
      rclcpp::QoS(1).best_effort());
    // Only phase and elapsed time cross domains; full match/info stays internal.
    forward<recruitment_sim_interfaces::msg::MatchStatus>(internal, external,
      "/referee_system/match/status", subscriptions, rclcpp::QoS(1).transient_local());

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = team_context;
    rclcpp::executors::SingleThreadedExecutor team_executor(executor_options);
    team_executor.add_node(external);
    // A single executor services both domains, avoiding extra worker threads.
    // Callback groups retain the context of their owning nodes.
    team_executor.add_node(internal);
    // SIGINT shuts down the default context; poll it while servicing both nodes.
    while (rclcpp::ok() && team_context->is_valid()) {
      team_executor.spin_once(std::chrono::milliseconds(20));
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("team_topic_bridge"), "%s", error.what());
    result = 1;
  }
  if (team_context->is_valid()) {team_context->shutdown("gateway stopped");}
  rclcpp::shutdown();
  return result;
}
