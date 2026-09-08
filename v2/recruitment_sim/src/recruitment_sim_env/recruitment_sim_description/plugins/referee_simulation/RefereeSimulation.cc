#include "EventBus.hh"
#include <chrono>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <map>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <sdf/Root.hh>
#include <sdf/Model.hh>

namespace recruitment_sim
{
using namespace ignition::gazebo;
class RefereeSimulation : public System, public ISystemConfigure,
  public ISystemPreUpdate, public ISystemPostUpdate
{
public:
  void Configure(const Entity & entity, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager & ecm, EventManager & events) override
  {
    world_ = entity; events_ = &events;
    creator_ = std::make_unique<SdfEntityCreator>(ecm, events);
    publisher_ = node_.Advertise<ignition::msgs::StringMsg>("/referee_system/simulation/frame");
    node_.Advertise("/referee_system/simulation/control", &RefereeSimulation::Command, this);
  }
  bool Command(const ignition::msgs::StringMsg & req, ignition::msgs::Boolean & res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (commands_.size() >= 16) {res.set_data(false); return true;}
    commands_.push_back(req.data()); res.set_data(true); return true;
  }
  Entity Find(const EntityComponentManager & ecm, const std::string & name) const
  {return ecm.EntityByComponents(components::Model(), components::Name(name), components::ParentEntity(world_));}
  void PreUpdate(const UpdateInfo & info, EntityComponentManager & ecm) override
  {
    std::deque<std::string> commands;
    {std::lock_guard<std::mutex> lock(mutex_); commands.swap(commands_);}
    for (const auto & command : commands) {
      std::istringstream in(command);
      std::string version, verb; uint64_t token, round;
      if (!(in >> version >> token >> round >> verb) || version != "V1" || token <= token_) {continue;}
      token_ = token;
      if (verb == "CONFIG") {
        size_t count; std::map<std::string, sdf::Model> parsed;
        if (!(in >> count) || count == 0 || count > 1000) {error_ = "invalid model count"; continue;}
        for (size_t i = 0; i < count; ++i) {
          std::string name, xml; sdf::Root root;
          if (!(in >> std::quoted(name) >> std::quoted(xml)) ||
            !root.LoadSdfString(xml).empty() || !root.Model()) {parsed.clear(); break;}
          parsed.emplace(name, *root.Model());
        }
        if (parsed.size() != count) {error_ = "invalid robot SDF configuration"; continue;}
        models_ = std::move(parsed); error_.clear(); phase_ = "READY";
      } else if (verb == "PAUSE") {
        phase_ = "PAUSED"; events_->Emit<events::Pause>(true);
      } else if (verb == "RESET" && info.paused && !models_.empty()) {
        SetRound(round); error_.clear(); phase_ = "REMOVING";
        // Fortress / Ogre2 cannot safely destroy and immediately recreate the
        // cameras and GPU lidars. Reset model state in place and keep sensors.
        for (const auto & model : models_) {
          auto entity = Find(ecm, model.first);
          if (entity == kNullEntity) {error_ = "robot missing during reset"; continue;}
          const auto pose = model.second.RawPose();
          ecm.SetComponentData<components::WorldPoseCmd>(entity, pose);
          const auto chassis = Model(entity).LinkByName(ecm, "chassis");
          ecm.SetComponentData<components::WorldLinearVelocityCmd>(chassis, {});
          ecm.SetComponentData<components::WorldAngularVelocityCmd>(chassis, {});
        }
        for (const auto e : TakeProjectiles()) {creator_->RequestRemoveEntity(e);}
        reset_cycles_ = 2; phase_ = "RESETTING";
      } else if (verb == "RESUME" && Ready(ecm)) {
        for (const auto & model : models_) {
          const auto entity = Find(ecm, model.first);
          const auto chassis = Model(entity).LinkByName(ecm, "chassis");
          ecm.RemoveComponent<components::WorldLinearVelocityCmd>(chassis);
          ecm.RemoveComponent<components::WorldAngularVelocityCmd>(chassis);
        }
        phase_ = "READY"; events_->Emit<events::Pause>(false);
      } else {error_ = "invalid simulation command or models not ready";}
    }
    if (phase_ == "RESETTING" && reset_cycles_ > 0 && --reset_cycles_ == 0 && error_.empty()) {
      phase_ = "READY";
    }
  }
  bool Ready(const EntityComponentManager & ecm) const
  {
    if (models_.empty()) {return false;}
    for (const auto & model : models_) {
      auto entity = Find(ecm, model.first);
      if (entity == kNullEntity || !ShooterReady(entity) ||
        Model(entity).LinkByName(ecm, "chassis") == kNullEntity) {return false;}
    }
    return true;
  }
  void PostUpdate(const UpdateInfo & info, const EntityComponentManager & ecm) override
  {
    const auto events = DrainEvents();
    const auto wall = std::chrono::steady_clock::now();
    if (info.paused && wall - last_wall_ < std::chrono::milliseconds(50)) {return;}
    last_wall_ = wall;
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    std::ostringstream out; out << std::setprecision(17);
    out << "V1 " << Round() << ' ' << stamp << ' ' << info.paused << ' ' << token_ << ' '
        << std::quoted(phase_) << ' ' << Ready(ecm) << ' ' << std::quoted(error_) << ' ';
    std::vector<std::pair<std::string, ignition::math::Pose3d>> poses;
    for (const auto & model : models_) {
      const auto entity = Find(ecm, model.first);
      if (entity == kNullEntity) {continue;}
      auto link = Model(entity).LinkByName(ecm, "chassis");
      if (link != kNullEntity) {poses.emplace_back(model.first, worldPose(link, ecm));}
    }
    out << poses.size();
    for (const auto & p : poses) {out << ' ' << std::quoted(p.first) << ' ' << p.second.Pos().X() << ' ' << p.second.Pos().Y();}
    out << ' ' << events.size();
    for (const auto & event : events) {out << ' ' << std::quoted(event);}
    ignition::msgs::StringMsg msg; msg.set_data(out.str()); publisher_.Publish(msg);
  }
private:
  Entity world_{kNullEntity};
  EventManager * events_{nullptr};
  std::unique_ptr<SdfEntityCreator> creator_;
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher publisher_;
  std::mutex mutex_;
  std::deque<std::string> commands_;
  std::map<std::string, sdf::Model> models_;
  uint64_t token_{0};
  std::string phase_{"UNCONFIGURED"}, error_;
  std::chrono::steady_clock::time_point last_wall_;
  unsigned int reset_cycles_{0};
};
}
IGNITION_ADD_PLUGIN(recruitment_sim::RefereeSimulation, ignition::gazebo::System,
  recruitment_sim::RefereeSimulation::ISystemConfigure,
  recruitment_sim::RefereeSimulation::ISystemPreUpdate,
  recruitment_sim::RefereeSimulation::ISystemPostUpdate)
