#include "recruitment_sim_referee_system/match_engine.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace recruitment_sim_referee_system
{
namespace {constexpr int64_t second = 1000000000LL; constexpr int64_t duration = 300 * second;}

MatchEngine::MatchEngine(std::vector<RobotConfig> configs, ZoneConfig zone)
: configs_(std::move(configs)), zone_(zone), referee_(configs_)
{
  if (!std::isfinite(zone.min_x) || !std::isfinite(zone.max_x) ||
    !std::isfinite(zone.min_y) || !std::isfinite(zone.max_y) ||
    zone.min_x >= zone.max_x || zone.min_y >= zone.max_y)
  {throw std::invalid_argument("invalid control zone bounds");}
}
int MatchEngine::team(const std::string & name) {return name == "red" ? 0 : name == "blue" ? 1 : -1;}
bool MatchEngine::reset(uint64_t round)
{
  if (state_ == RESETTING || state_ == ENDING || round <= round_) {return false;}
  // Non-team robots remain useful for training, but cannot enter a scored match.
  for (const auto & config : configs_) {if (team(config.team) < 0) {return false;}}
  round_ = round;
  state_ = RESETTING;
  result_ = NONE;
  reason_.clear(); error_.clear();
  return true;
}
void MatchEngine::reset_complete()
{
  if (state_ != RESETTING) {return;}
  referee_ = RefereeEngine(configs_);
  points_ = {{200, 200}}; occupied_ns_ = {{0, 0}};
  occupants_.clear(); owner_ = -1; tied_entry_ = false;
  start_ = 0; last_ = -1; cooling_ = 0;
  state_ = READY;
}
bool MatchEngine::start(int64_t stamp)
{
  if (state_ != READY) {return false;}
  start_ = last_ = cooling_ = stamp;
  state_ = RUNNING;
  return true;
}
void MatchEngine::end()
{
  if (state_ == FINISHED || state_ == ENDING) {return;}
  result_ = ABORTED; reason_ = "manual_end"; state_ = ENDING;
}
void MatchEngine::paused() {if (state_ == ENDING) {state_ = FINISHED;}}
void MatchEngine::fail(const std::string & reason) {error_ = reason; state_ = ERROR;}
double MatchEngine::elapsed() const
{
  if (state_ == TRAINING || state_ == STARTING || state_ == READY || state_ == RESETTING) {return 0;}
  return std::max<int64_t>(0, std::min(duration, last_ - start_)) / double(second);
}
std::array<int, 2> MatchEngine::hp() const
{
  std::array<int, 2> sums{{0, 0}};
  for (const auto & r : referee_.robots()) {
    const int t = team(r.second.config.team); if (t >= 0) {sums[t] += r.second.current_hp;}
  }
  return sums;
}
std::array<uint64_t, 2> MatchEngine::damage() const
{
  std::array<uint64_t, 2> sums{{0, 0}};
  for (const auto & r : referee_.robots()) {
    const int t = team(r.second.config.team); if (t >= 0) {sums[t] += r.second.attack_damage;}
  }
  return sums;
}
std::vector<std::string> MatchEngine::eligible() const
{
  std::vector<std::string> names;
  for (const auto & entry : occupants_) {names.push_back(entry.first);}
  return names;
}
void MatchEngine::choose_owner()
{
  const auto infinity = std::numeric_limits<int64_t>::max();
  std::array<int64_t, 2> first{{infinity, infinity}};
  for (const auto & o : occupants_) {
    int t = team(referee_.robots().at(o.first).config.team);
    if (t >= 0) {first[t] = std::min(first[t], o.second.entered);}
  }
  if (owner_ >= 0 && first[owner_] != infinity) {return;}
  owner_ = -1;
  if (first[0] == infinity || first[1] == infinity) {
    tied_entry_ = false;
    if (first[0] != infinity) {owner_ = 0;}
    if (first[1] != infinity) {owner_ = 1;}
  } else if (first[0] == first[1] || tied_entry_) {tied_entry_ = true;}
  else {owner_ = first[0] < first[1] ? 0 : 1;}
}
void MatchEngine::advance(int64_t stamp)
{
  // Split long frames at grace expirations; never charge an expired occupant.
  while (last_ < stamp) {
    int64_t next = stamp;
    for (const auto & o : occupants_) {next = std::min(next, o.second.expires);}
    if (owner_ >= 0) {
      occupied_ns_[owner_] += next - last_;
      const auto whole = occupied_ns_[owner_] / second;
      points_[1 - owner_] = std::max<int64_t>(0, points_[1 - owner_] - whole);
      occupied_ns_[owner_] %= second;
    }
    last_ = next;
    for (auto it = occupants_.begin(); it != occupants_.end();) {
      if (it->second.expires <= last_) {it = occupants_.erase(it);} else {++it;}
    }
    choose_owner();
  }
}
void MatchEngine::settle(bool timeout)
{
  if (points_[0] == 0 || points_[1] == 0) {
    result_ = points_[0] == points_[1] ? DRAW : points_[0] > 0 ? RED_WIN : BLUE_WIN;
    reason_ = "victory_points_zero";
  } else if (timeout) {
    const auto d = damage(); const auto h = hp();
    int winner = points_[0] != points_[1] ? (points_[0] > points_[1] ? 0 : 1) :
      d[0] != d[1] ? (d[0] > d[1] ? 0 : 1) :
      h[0] != h[1] ? (h[0] > h[1] ? 0 : 1) : -1;
    result_ = winner < 0 ? DRAW : winner == 0 ? RED_WIN : BLUE_WIN;
    reason_ = "time_limit";
  }
  if (result_ != NONE) {state_ = ENDING;}
}
void MatchEngine::process(const MatchFrame & frame)
{
  if ((state_ != TRAINING && state_ != READY && state_ != RUNNING) || frame.round != round_ ||
    frame.stamp <= last_) {return;}
  const bool running = state_ == RUNNING;
  if (last_ < 0) {last_ = cooling_ = frame.stamp;}
  const int64_t stamp = running ? std::min(frame.stamp, start_ + duration) : frame.stamp;
  if (running) {advance(stamp);} else {last_ = stamp;}
  // Events belong to a complete physics frame, not independent transport callbacks.
  if (!running || frame.stamp <= start_ + duration) {
    for (const auto & s : frame.shots) {referee_.process_shot(s);}
    for (const auto & h : frame.hits) {
      const auto before = referee_.robots().find(h.target);
      const bool alive = before != referee_.robots().end() && before->second.alive;
      if (referee_.process_hit(h) && running && alive && !referee_.robots().at(h.target).alive) {
        const int t = team(referee_.robots().at(h.target).config.team);
        if (t >= 0) {points_[t] = std::max(0, points_[t] - 20);}
      }
    }
  }
  while (cooling_ + second / 10 <= stamp) {referee_.cool_one_period(); cooling_ += second / 10;}
  if (!running) {return;}
  std::set<std::string> inside;
  if (zone_.enabled) {
    for (const auto & p : frame.positions) {
      const auto r = referee_.robots().find(p.name);
      if (r != referee_.robots().end() && r->second.alive &&
        p.x >= zone_.min_x && p.x <= zone_.max_x && p.y >= zone_.min_y && p.y <= zone_.max_y)
      {
        inside.insert(p.name);
        if (!occupants_.count(p.name)) {occupants_[p.name] = {stamp, std::numeric_limits<int64_t>::max()};}
        occupants_[p.name].expires = std::numeric_limits<int64_t>::max();
      }
    }
  }
  for (auto it = occupants_.begin(); it != occupants_.end();) {
    if (!referee_.robots().at(it->first).alive) {it = occupants_.erase(it); continue;}
    if (!inside.count(it->first) && it->second.expires == std::numeric_limits<int64_t>::max()) {
      it->second.expires = stamp + 2 * second;
    }
    ++it;
  }
  choose_owner();
  settle(stamp >= start_ + duration);
}
}  // namespace recruitment_sim_referee_system
