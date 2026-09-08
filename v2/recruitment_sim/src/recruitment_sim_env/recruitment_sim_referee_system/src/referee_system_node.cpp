#include "recruitment_sim_referee_system/referee_engine.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ignition/msgs/stringmsg.pb.h"
#include "ignition/transport/Node.hh"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "recruitment_sim_interfaces/msg/robot_status.hpp"
#include "recruitment_sim_interfaces/srv/set_robot_enabled.hpp"

namespace recruitment_sim_referee_system
{

namespace
{
std::vector<std::string> split(const std::string & input)
{
  std::vector<std::string> fields;
  std::stringstream stream(input);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::int64_t stamp_ns(const ignition::msgs::StringMsg & msg)
{
  return static_cast<std::int64_t>(msg.header().stamp().sec()) * 1000000000LL +
         msg.header().stamp().nsec();
}
}  // namespace

class RefereeSystemNode : public rclcpp::Node
{
public:
  RefereeSystemNode()
  : Node("referee_system")
  {
    const auto names = declare_parameter(
      "robot_names", std::vector<std::string>{});
    const auto teams = declare_parameter(
      "robot_teams", std::vector<std::string>{});
    const auto max_hps = declare_parameter(
      "robot_max_hps", std::vector<std::int64_t>{});
    const auto heat_limits = declare_parameter(
      "robot_heat_limits", std::vector<double>{});
    const auto cooling_rates = declare_parameter(
      "robot_cooling_rates", std::vector<double>{});
    const auto count = names.size();
    if (count == 0 || teams.size() != count || max_hps.size() != count ||
      heat_limits.size() != count || cooling_rates.size() != count)
    {
      throw std::runtime_error("robot referee parameter arrays must be non-empty and equal length");
    }

    std::vector<RobotConfig> configs;
    configs.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      configs.push_back(
        {names[i], teams[i], static_cast<int>(max_hps[i]), heat_limits[i], cooling_rates[i]});
    }
    engine_ = std::make_unique<RefereeEngine>(configs);

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    for (const auto & config : configs) {
      status_publishers_[config.name] =
        create_publisher<recruitment_sim_interfaces::msg::RobotStatus>(
        "/referee_system/" + config.name + "/status", status_qos);
      enable_clients_[config.name] =
        create_client<recruitment_sim_interfaces::srv::SetRobotEnabled>(
        "/referee_system/" + config.name + "/set_enabled");
    }

    if (!gz_node_.Subscribe(
        "/referee_system/events/shot", &RefereeSystemNode::shot_callback, this))
    {
      throw std::runtime_error("failed to subscribe to Gazebo shot events");
    }
    if (!gz_node_.Subscribe(
        "/referee_system/events/hit", &RefereeSystemNode::hit_callback, this))
    {
      throw std::runtime_error("failed to subscribe to Gazebo hit events");
    }

    settlement_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(100),
      std::bind(&RefereeSystemNode::settlement_callback, this));
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&RefereeSystemNode::control_callback, this));
    RCLCPP_INFO(get_logger(), "referee system initialized for %zu robots", count);
  }

private:
  struct DesiredControl
  {
    bool desired{true};
    bool applied{true};
    bool has_applied{false};
    bool in_flight{false};
    std::int64_t next_try_ns{0};
  };

  using EnableService = recruitment_sim_interfaces::srv::SetRobotEnabled;
  using ControlKey = std::pair<std::string, std::uint8_t>;

  void shot_callback(const ignition::msgs::StringMsg & msg)
  {
    const auto fields = split(msg.data());
    if (fields.size() != 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ignored malformed Gazebo shot event");
      return;
    }
    try {
      ShotEvent event{fields[0], std::stoull(fields[2]), fields[3], stamp_ns(msg)};
      std::lock_guard<std::mutex> lock(engine_mutex_);
      engine_->process_shot(event);
    } catch (const std::exception &) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ignored malformed projectile id in shot event");
    }
  }

  void hit_callback(const ignition::msgs::StringMsg & msg)
  {
    const auto fields = split(msg.data());
    if (fields.size() != 6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ignored malformed Gazebo hit event");
      return;
    }
    try {
      HitEvent event{
        fields[0], std::stoull(fields[2]), fields[3], fields[4], fields[5], stamp_ns(msg)};
      std::lock_guard<std::mutex> lock(engine_mutex_);
      engine_->process_hit(event);
    } catch (const std::exception &) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ignored malformed projectile id in hit event");
    }
  }

  void settlement_callback()
  {
    std::map<std::string, RobotState> states;
    {
      std::lock_guard<std::mutex> lock(engine_mutex_);
      engine_->cool_one_period();
      states = engine_->robots();
    }

    const auto now = get_clock()->now();
    for (const auto & item : states) {
      const auto & state = item.second;
      recruitment_sim_interfaces::msg::RobotStatus msg;
      msg.header.stamp = now;
      msg.robot_name = state.config.name;
      msg.max_hp = state.config.max_hp;
      msg.current_hp = state.current_hp;
      msg.shooter_heat = state.heat;
      msg.heat_limit = state.config.heat_limit;
      msg.cooling_rate = state.config.cooling_rate;
      msg.shots_last_period = state.shots_last_period;
      msg.total_shots = state.total_shots;
      msg.total_hits = state.total_hits;
      msg.alive = state.alive;
      msg.shooter_overheated = state.overheated;
      msg.shooter_permanently_locked = state.permanently_locked;
      status_publishers_.at(state.config.name)->publish(msg);
    }
  }

  void control_callback()
  {
    std::vector<ControlCommand> commands;
    {
      std::lock_guard<std::mutex> lock(engine_mutex_);
      commands = engine_->take_control_commands();
    }
    for (const auto & command : commands) {
      const ControlKey key{command.robot_name, static_cast<std::uint8_t>(command.target)};
      auto & control = desired_controls_[key];
      control.desired = command.enabled;
      control.has_applied = false;
      control.next_try_ns = 0;
    }
    dispatch_controls();
  }

  static std::int64_t steady_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void dispatch_controls()
  {
    const auto now_ns = steady_now_ns();
    for (auto & item : desired_controls_) {
      const auto key = item.first;
      auto & control = item.second;
      if (control.in_flight ||
        (control.has_applied && control.applied == control.desired) ||
        now_ns < control.next_try_ns)
      {
        continue;
      }
      auto client = enable_clients_.at(key.first);
      if (!client->service_is_ready()) {
        control.next_try_ns = now_ns + 1000000000LL;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "waiting for enable service of robot %s", key.first.c_str());
        continue;
      }

      auto request = std::make_shared<EnableService::Request>();
      request->target = key.second;
      request->enabled = control.desired;
      const bool sent_value = control.desired;
      control.in_flight = true;
      client->async_send_request(
        request,
        [this, key, sent_value](rclcpp::Client<EnableService>::SharedFuture future) {
          auto & current = desired_controls_.at(key);
          current.in_flight = false;
          try {
            const auto response = future.get();
            if (response->success) {
              current.applied = sent_value;
              current.has_applied = true;
            } else {
              current.has_applied = false;
              current.next_try_ns = steady_now_ns() + 1000000000LL;
              RCLCPP_WARN(
                get_logger(), "enable request rejected for %s: %s",
                key.first.c_str(), response->message.c_str());
            }
          } catch (const std::exception & error) {
            current.has_applied = false;
            current.next_try_ns = steady_now_ns() + 1000000000LL;
            RCLCPP_WARN(
              get_logger(), "enable request failed for %s: %s",
              key.first.c_str(), error.what());
          }
        });
    }
  }

  std::unique_ptr<RefereeEngine> engine_;
  std::mutex engine_mutex_;
  ignition::transport::Node gz_node_;
  std::map<std::string,
    rclcpp::Publisher<recruitment_sim_interfaces::msg::RobotStatus>::SharedPtr>
  status_publishers_;
  std::map<std::string, rclcpp::Client<EnableService>::SharedPtr> enable_clients_;
  std::map<ControlKey, DesiredControl> desired_controls_;
  rclcpp::TimerBase::SharedPtr settlement_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace recruitment_sim_referee_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<recruitment_sim_referee_system::RefereeSystemNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("referee_system"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
