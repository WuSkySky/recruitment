#pragma once

#include "recruitment_sim_referee_system/referee_engine.hpp"
#include <array>

namespace recruitment_sim_referee_system
{
struct ZoneConfig
{
  bool enabled{true};
  double min_x{-1.5}, max_x{1.5}, min_y{-1.5}, max_y{1.5};
};
struct RobotPosition {std::string name; double x, y;};
struct MatchFrame
{
  uint64_t round{0};
  int64_t stamp{0};
  std::vector<RobotPosition> positions;
  std::vector<ShotEvent> shots;
  std::vector<HitEvent> hits;
};
class MatchEngine
{
public:
  enum State : uint8_t {TRAINING, STARTING, RUNNING, ENDING, FINISHED, ERROR, READY, RESETTING};
  enum Result : uint8_t {NONE, RED_WIN, BLUE_WIN, DRAW, ABORTED};
  explicit MatchEngine(std::vector<RobotConfig> configs, ZoneConfig zone = {});
  bool reset(uint64_t round);
  void reset_complete();
  bool start(int64_t stamp);
  void end();
  void paused();
  void fail(const std::string & reason);
  void process(const MatchFrame & frame);
  RefereeEngine & referee() {return referee_;}
  const RefereeEngine & referee() const {return referee_;}
  State state() const {return state_;}
  Result result() const {return result_;}
  uint64_t round() const {return round_;}
  int owner() const {return owner_;}
  const std::array<int, 2> & points() const {return points_;}
  std::array<uint64_t, 2> damage() const;
  std::array<int, 2> hp() const;
  std::vector<std::string> eligible() const;
  double elapsed() const;
  const std::string & reason() const {return reason_;}
  const std::string & error() const {return error_;}
private:
  struct Occupant {int64_t entered; int64_t expires;};
  static int team(const std::string & name);
  void choose_owner();
  void advance(int64_t stamp);
  void settle(bool timeout);
  std::vector<RobotConfig> configs_;
  ZoneConfig zone_;
  RefereeEngine referee_;
  State state_{TRAINING};
  Result result_{NONE};
  uint64_t round_{0};
  int64_t start_{0}, last_{-1}, cooling_{0};
  std::array<int, 2> points_{{200, 200}};
  std::array<int64_t, 2> occupied_ns_{{0, 0}};
  std::map<std::string, Occupant> occupants_;
  int owner_{-1};
  bool tied_entry_{false};
  std::string reason_, error_;
};
}  // namespace recruitment_sim_referee_system
