#include "recruitment_sim_referee_system/match_engine.hpp"
#include <chrono>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include "recruitment_sim_interfaces/msg/robot_status.hpp"
#include "recruitment_sim_interfaces/msg/match_status.hpp"
#include "recruitment_sim_interfaces/srv/set_robot_enabled.hpp"
#include "recruitment_sim_interfaces/srv/reset_robot.hpp"
#include "recruitment_sim_interfaces/srv/control_match.hpp"

namespace recruitment_sim_referee_system
{
using Enable = recruitment_sim_interfaces::srv::SetRobotEnabled;
using Reset = recruitment_sim_interfaces::srv::ResetRobot;
using Control = recruitment_sim_interfaces::srv::ControlMatch;
using Status = recruitment_sim_interfaces::msg::MatchStatus;
using RobotStatus = recruitment_sim_interfaces::msg::RobotStatus;
class RefereeSystemNode : public rclcpp::Node
{
public:
  RefereeSystemNode() : Node("referee_system")
  {
    const auto names = declare_parameter("robot_names", std::vector<std::string>{});
    const auto teams = declare_parameter("robot_teams", std::vector<std::string>{});
    const auto hp = declare_parameter("robot_max_hps", std::vector<int64_t>{});
    const auto heat = declare_parameter("robot_heat_limits", std::vector<double>{});
    const auto cool = declare_parameter("robot_cooling_rates", std::vector<double>{});
    const auto sdfs = declare_parameter("robot_sdfs", std::vector<std::string>{});
    const auto bounds = declare_parameter("zone_bounds", std::vector<double>{-1.5, 1.5, -1.5, 1.5});
    const bool zone = declare_parameter("zone_enabled", true);
    if (names.empty() || teams.size() != names.size() || hp.size() != names.size() ||
      heat.size() != names.size() || cool.size() != names.size() || sdfs.size() != names.size() ||
      bounds.size() != 4) {throw std::runtime_error("invalid referee configuration arrays");}
    std::vector<RobotConfig> configs;
    std::ostringstream config; config << names.size();
    auto qos = rclcpp::QoS(10).reliable().transient_local();
    for (size_t i = 0; i < names.size(); ++i) {
      configs.push_back({names[i], teams[i], static_cast<int>(hp[i]), heat[i], cool[i]});
      config << ' ' << std::quoted(names[i]) << ' ' << std::quoted(sdfs[i]);
      publishers_[names[i]] = create_publisher<RobotStatus>("/referee_system/" + names[i] + "/status", qos);
      enables_[names[i]] = create_client<Enable>("/referee_system/" + names[i] + "/set_enabled");
      resets_[names[i]] = create_client<Reset>("/referee_system/" + names[i] + "/reset");
    }
    match_ = std::make_unique<MatchEngine>(configs, ZoneConfig{zone, bounds[0], bounds[1], bounds[2], bounds[3]});
    publisher_ = create_publisher<Status>("/referee_system/match/status", qos);
    service_ = create_service<Control>("/referee_system/match/control",
      [this](const std::shared_ptr<Control::Request> req, std::shared_ptr<Control::Response> res) {
        if (req->command == Control::Request::START) {
          if (stage_ != Stage::IDLE || !match_->start(last_stamp_)) {
            res->message = "match is not READY";
          }
          else {
            res->accepted = true; res->message = "match started";
          }
        } else if (req->command == Control::Request::END) {
          res->accepted = true; res->message = "end accepted";
          if (match_->state() != MatchEngine::FINISHED && match_->state() != MatchEngine::ENDING) {
            match_->end(); begin_end();
          }
        } else if (req->command == Control::Request::RESUME) {
          if (!match_->reset(++serial_)) {
            res->message = "reset is already running or ending";
          } else {
            new_operation();
            if (!configured_) {simulation("CONFIG", Stage::CONFIG, config_payload_);}
            else {begin_base(false, Stage::START_STOP);}
            res->accepted = true; res->message = "reset and resume accepted; wait for READY";
          }
        } else {res->message = "unknown match command";}
        res->round_id = match_->round(); publish();
      });
    node_.Subscribe("/referee_system/simulation/frame", &RefereeSystemNode::receive, this);
    serial_ = static_cast<uint64_t>(steady());
    config_payload_ = config.str();
    simulation("CONFIG", Stage::CONFIG, config_payload_);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {tick();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {publish();});
    RCLCPP_INFO(get_logger(), "referee training mode, %zu robots; RESUME resets and START begins judging", names.size());
  }
private:
  enum class Stage {CONFIG, IDLE, START_STOP, START_PAUSE, RESET, ENABLE, RESUME, END_STOP, END_PAUSE};
  struct Frame {
    MatchFrame game;
    bool paused{true}, ready{false};
    uint64_t token{0};
    std::string phase, error;
  };
  struct Pending {
    bool acknowledged{false}, in_flight{false};
    int64_t sent{0}, id{0};
  };
  struct Desired : Pending {bool value{false};};
  static int64_t steady() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void receive(const ignition::msgs::StringMsg & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames_.size() >= 20000) {overflow_ = true; return;}
    frames_.push_back(message.data());
  }
  bool parse(const std::string & text, Frame & f)
  {
    std::istringstream in(text); std::string version; size_t count;
    if (!(in >> version >> f.game.round >> f.game.stamp >> f.paused >> f.token >>
      std::quoted(f.phase) >> f.ready >> std::quoted(f.error) >> count) ||
      version != "V1" || count > 1000) {return false;}
    for (size_t i = 0; i < count; ++i) {
      RobotPosition p;
      if (!(in >> std::quoted(p.name) >> p.x >> p.y)) {return false;}
      f.game.positions.push_back(p);
    }
    if (!(in >> count) || count > 10000) {return false;}
    for (size_t i = 0; i < count; ++i) {
      std::string event; if (!(in >> std::quoted(event))) {return false;}
      std::istringstream e(event); char kind; e >> kind;
      if (kind == 'S') {
        ShotEvent shot; shot.stamp_ns = f.game.stamp; shot.projectile_type = "17mm";
        if (!(e >> std::quoted(shot.shooter) >> shot.projectile_id)) {return false;}
        f.game.shots.push_back(shot);
      } else if (kind == 'H') {
        HitEvent hit; hit.stamp_ns = f.game.stamp;
        if (!(e >> std::quoted(hit.shooter) >> hit.projectile_id >> std::quoted(hit.target) >>
          std::quoted(hit.target_link) >> std::quoted(hit.target_collision))) {return false;}
        f.game.hits.push_back(hit);
      } else {return false;}
    }
    return true;
  }
  void new_operation()
  {
    control_epoch_ = ++serial_;
    for (auto & p : pending_) {if (p.second.in_flight) {resets_.at(p.first)->remove_pending_request(p.second.id);}}
    for (auto & p : desired_) {if (p.second.in_flight) {enables_.at(p.first.first)->remove_pending_request(p.second.id);}}
    pending_.clear(); desired_.clear();
    match_->referee().take_control_commands();
  }
  void begin_base(bool enabled, Stage stage)
  {
    stage_ = stage; base_value_ = enabled; stage_since_ = steady(); pending_.clear();
    for (const auto & client : resets_) {pending_[client.first] = Pending{};}
  }
  void simulation(const std::string & verb, Stage stage, const std::string & extra = "")
  {
    stage_ = stage; stage_since_ = steady(); token_ = ++serial_; last_send_ = 0;
    std::ostringstream s; s << "V1 " << token_ << ' ' << match_->round() << ' ' << verb << ' ' << extra;
    simulation_command_ = s.str();
  }
  void begin_end()
  {
    new_operation(); begin_base(false, Stage::END_STOP); publish();
  }
  void fail(const std::string & message)
  {
    match_->fail(message); new_operation(); stage_ = Stage::IDLE;
    // Best effort fail-safe only: ERROR never claims pause/disable succeeded.
    for (const auto & c : resets_) {
      auto req = std::make_shared<Reset::Request>(); req->round_id = control_epoch_; req->enabled = false;
      if (c.second->service_is_ready()) {c.second->async_send_request(req,
        [](rclcpp::Client<Reset>::SharedFuture) {});}
    }
    ignition::msgs::StringMsg req;
    req.set_data("V1 " + std::to_string(++serial_) + " " + std::to_string(match_->round()) + " PAUSE");
    node_.Request("/referee_system/simulation/control", req,
      &RefereeSystemNode::ignore_ack);
    RCLCPP_ERROR(get_logger(), "%s", message.c_str()); publish();
  }
  void tick()
  {
    std::deque<std::string> frames; bool overflow;
    {std::lock_guard<std::mutex> lock(mutex_); frames.swap(frames_); overflow = overflow_; overflow_ = false;}
    if (overflow) {fail("simulation frame queue overflow; match invalid"); return;}
    for (const auto & text : frames) {
      Frame frame; if (!parse(text, frame)) {fail("malformed simulation frame"); return;}
      last_frame_wall_ = steady();
      last_stamp_ = frame.game.stamp;
      if (frame.token == token_) {
        if (!frame.error.empty()) {fail(frame.error); return;}
        if (stage_ == Stage::CONFIG && frame.ready) {
          configured_ = true; stage_ = Stage::IDLE;
          if (match_->state() == MatchEngine::RESETTING) {begin_base(false, Stage::START_STOP);}
        }
        else if (stage_ == Stage::START_PAUSE && frame.paused) {simulation("RESET", Stage::RESET);}
        else if (stage_ == Stage::RESET && frame.ready && frame.phase == "READY" &&
          frame.game.round == match_->round()) {begin_base(true, Stage::ENABLE);}
        else if (stage_ == Stage::RESUME && !frame.paused && frame.ready) {
          match_->reset_complete();
          if (match_->state() != MatchEngine::READY) {fail("match left RESETTING while resuming"); return;}
          stage_ = Stage::IDLE; publish();
        } else if (stage_ == Stage::END_PAUSE && frame.paused) {
          match_->paused(); stage_ = Stage::IDLE; publish();
        }
      }
      if (!frame.paused) {
        const auto before = match_->state();
        match_->process(frame.game);
        if (before == MatchEngine::RUNNING && match_->state() == MatchEngine::ENDING) {begin_end();}
      }
    }
    if (stage_ != Stage::IDLE && steady() - stage_since_ > 10000000000LL) {
      fail("lifecycle stage " + std::to_string(static_cast<int>(stage_)) + " timed out (10 s)"); return;
    }
    if (match_->state() == MatchEngine::RUNNING && last_frame_wall_ &&
      steady() - last_frame_wall_ > 10000000000LL) {fail("simulation heartbeat lost"); return;}
    if (stage_ == Stage::START_STOP || stage_ == Stage::ENABLE || stage_ == Stage::END_STOP) {
      dispatch_reset();
      bool done = true;
      for (const auto & p : pending_) {done = done && p.second.acknowledged;}
      if (done) {
        if (stage_ == Stage::START_STOP) {simulation("PAUSE", Stage::START_PAUSE);}
        else if (stage_ == Stage::ENABLE) {simulation("RESUME", Stage::RESUME);}
        else {simulation("PAUSE", Stage::END_PAUSE);}
      }
    } else if (stage_ != Stage::IDLE && steady() - last_send_ >= 1000000000LL) {
      ignition::msgs::StringMsg req; req.set_data(simulation_command_); last_send_ = steady();
      node_.Request("/referee_system/simulation/control", req,
        &RefereeSystemNode::ignore_ack);
    }
    if (stage_ == Stage::IDLE &&
      (match_->state() == MatchEngine::TRAINING || match_->state() == MatchEngine::READY ||
      match_->state() == MatchEngine::RUNNING)) {
      for (const auto & command : match_->referee().take_control_commands()) {
        const auto key = std::make_pair(command.robot_name, static_cast<uint8_t>(command.target));
        auto & d = desired_[key];
        if (d.value != command.enabled || !d.acknowledged) {
          d.value = command.enabled; d.acknowledged = false;
        }
      }
      dispatch_enable();
    }
  }
  void dispatch_reset()
  {
    for (auto & p : pending_) {
      auto client = resets_.at(p.first); auto & pending = p.second;
      if (pending.acknowledged || steady() - pending.sent < 1000000000LL) {continue;}
      if (pending.in_flight) {client->remove_pending_request(pending.id); pending.in_flight = false;}
      if (!client->service_is_ready()) {continue;}
      auto req = std::make_shared<Reset::Request>();
      req->round_id = control_epoch_; req->enabled = base_value_;
      const auto epoch = control_epoch_; const auto stage = stage_; const auto name = p.first;
      pending.sent = steady(); pending.in_flight = true;
      pending.id = client->async_send_request(req,
        [this, epoch, stage, name](rclcpp::Client<Reset>::SharedFuture future) {
          if (epoch != control_epoch_ || stage != stage_) {return;}
          auto & p = pending_.at(name); p.in_flight = false;
          try {p.acknowledged = future.get()->success;} catch (const std::exception &) {}
        }).request_id;
    }
  }
  static void ignore_ack(const ignition::msgs::Boolean &, bool) {}
  void dispatch_enable()
  {
    for (auto & item : desired_) {
      auto & d = item.second; auto client = enables_.at(item.first.first);
      if (d.acknowledged || steady() - d.sent < 1000000000LL) {continue;}
      if (d.in_flight) {client->remove_pending_request(d.id); d.in_flight = false;}
      if (!client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for base enable service");
        continue;
      }
      auto req = std::make_shared<Enable::Request>();
      req->target = item.first.second; req->enabled = d.value; req->round_id = control_epoch_;
      auto key = item.first; auto epoch = control_epoch_; bool value = d.value;
      d.sent = steady(); d.in_flight = true;
      d.id = client->async_send_request(req,
        [this, key, epoch, value](rclcpp::Client<Enable>::SharedFuture future) {
          if (epoch != control_epoch_ || !desired_.count(key)) {return;}
          auto & d = desired_.at(key); d.in_flight = false;
          try {d.acknowledged = future.get()->success && d.value == value;} catch (const std::exception &) {}
        }).request_id;
    }
  }
  void publish()
  {
    Status s; s.header.stamp = now(); s.round_id = match_->round(); s.state = match_->state();
    s.elapsed_seconds = match_->elapsed(); s.remaining_seconds = 300 - s.elapsed_seconds;
    s.red_victory_points = match_->points()[0]; s.blue_victory_points = match_->points()[1];
    auto d = match_->damage(); auto h = match_->hp();
    s.red_attack_damage = d[0]; s.blue_attack_damage = d[1];
    s.red_remaining_hp = h[0]; s.blue_remaining_hp = h[1];
    s.control_zone_owner = match_->owner() < 0 ? "" : match_->owner() == 0 ? "red" : "blue";
    s.eligible_robots = match_->eligible(); s.result = match_->result();
    s.end_reason = match_->reason(); s.error_message = match_->error(); publisher_->publish(s);
    for (const auto & item : match_->referee().robots()) {
      const auto & r = item.second; RobotStatus msg; msg.header.stamp = s.header.stamp;
      msg.robot_name = item.first; msg.max_hp = r.config.max_hp; msg.current_hp = r.current_hp;
      msg.shooter_heat = r.heat; msg.heat_limit = r.config.heat_limit; msg.cooling_rate = r.config.cooling_rate;
      msg.shots_last_period = r.shots_last_period; msg.total_shots = r.total_shots; msg.total_hits = r.total_hits;
      msg.alive = r.alive; msg.shooter_overheated = r.overheated; msg.shooter_permanently_locked = r.permanently_locked;
      publishers_.at(item.first)->publish(msg);
    }
  }
  std::unique_ptr<MatchEngine> match_;
  ignition::transport::Node node_;
  std::mutex mutex_;
  std::deque<std::string> frames_;
  bool overflow_{false}, configured_{false}, base_value_{false};
  Stage stage_{Stage::IDLE};
  uint64_t serial_{0}, token_{0}, control_epoch_{0};
  int64_t stage_since_{0}, last_send_{0}, last_stamp_{0}, last_frame_wall_{0};
  std::string simulation_command_;
  std::string config_payload_;
  std::map<std::string, rclcpp::Publisher<RobotStatus>::SharedPtr> publishers_;
  rclcpp::Publisher<Status>::SharedPtr publisher_;
  rclcpp::Service<Control>::SharedPtr service_;
  std::map<std::string, rclcpp::Client<Enable>::SharedPtr> enables_;
  std::map<std::string, rclcpp::Client<Reset>::SharedPtr> resets_;
  std::map<std::string, Pending> pending_;
  std::map<std::pair<std::string, uint8_t>, Desired> desired_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<recruitment_sim_referee_system::RefereeSystemNode>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("referee_system"), "%s", e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
