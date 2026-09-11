// Copyright 2026 Recruitment Simulation Maintainers
// SPDX-License-Identifier: Apache-2.0
#include "Runtime.hh"
#include <gazebo/gazebo.hh>
#include <recruitment_sim_interfaces/msg/simulation_frame.hpp>
#include <recruitment_sim_interfaces/srv/control_simulation.hpp>
namespace recruitment_sim {
using Frame = recruitment_sim_interfaces::msg::SimulationFrame;
using Control = recruitment_sim_interfaces::srv::ControlSimulation;
class World : public gazebo::WorldPlugin {
public:
  ~World() override {
    end_.reset();
    {std::lock_guard<std::recursive_mutex> lock(State().mutex); *alive_=false;}
    if(domain_) domain_->Remove(node_);
  }
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr) override {
    world_=world; domain_=GetDomain(InternalDomain()); node_=domain_->Node("simulation", "/referee_system");
    publisher_=node_->create_publisher<Frame>("simulation/frame",rclcpp::QoS(2000).reliable());
    auto alive=alive_;
    service_=node_->create_service<Control>("simulation/control",
      [this,alive](Control::Request::SharedPtr req, Control::Response::SharedPtr res) {
        std::unique_lock<std::recursive_mutex> lock(State().mutex);
        if(!*alive) return;
        if(req->token<token_) {res->message="stale token"; return;}
        if(req->token==token_) {res->success=error_.empty(); res->message=error_; return;}
        token_=req->token; error_.clear();
        if(req->operation==Control::Request::CONFIG) {
          names_=req->robot_names;
          if(names_.empty()) error_="empty robot configuration";
          phase_="READY";
        } else if(req->operation==Control::Request::PAUSE) {
          lock.unlock(); world_->SetPaused(true); lock.lock(); phase_="PAUSED";
        } else if(req->operation==Control::Request::RESET && world_->IsPaused() && Ready()) {
          State().round=req->round_id; State().events.clear();
          for(const auto & name:State().projectiles) world_->RemoveModel(name);
          phase_="RESETTING";
          for(const auto & name:names_) {
            auto & robot=State().robots.at(name);
            robot.model->SetWorldPose(robot.spawn); robot.reset();
          }
        } else if(req->operation==Control::Request::RESUME && phase_=="READY" && Ready()) {
          lock.unlock(); world_->SetPaused(false); lock.lock();
        } else error_="invalid command, unpaused reset, or robots not ready";
        res->success=error_.empty(); res->message=error_;
        Publish();
      });
    end_=gazebo::event::Events::ConnectWorldUpdateEnd([this,alive] {
      std::lock_guard<std::recursive_mutex> lock(State().mutex);
      if(*alive) {for(auto & item:State().robots) item.second.contacts(); Publish();}
    });
    timer_=node_->create_wall_timer(std::chrono::milliseconds(50),[this,alive] {
      std::lock_guard<std::recursive_mutex> lock(State().mutex);
      if(!*alive) return;
      if(phase_=="RESETTING") {
        bool cleared=true;
        for(const auto & name:State().projectiles) {
          if(world_->ModelByName(name)) {world_->RemoveModel(name); cleared=false;}
        }
        if(cleared) {State().projectiles.clear(); phase_="READY";}
      }
      if(world_->IsPaused() || !Ready()) Publish();
    });
  }
private:
  bool Ready() const {
    if(names_.empty()) return false;
    for(const auto & name:names_) if(!State().robots.count(name)) return false;
    return true;
  }
  void Publish() {
    Frame frame; frame.round_id=State().round; frame.stamp_ns=Nanoseconds(world_->SimTime());
    frame.paused=world_->IsPaused(); frame.token=token_; frame.phase=phase_;
    frame.ready=Ready() && phase_!="RESETTING"; frame.error=error_;
    for(const auto & name:names_) {
      auto it=State().robots.find(name); if(it==State().robots.end()) continue;
      auto pose=it->second.model->GetLink("chassis")->WorldPose();
      recruitment_sim_interfaces::msg::SimulationPosition p; p.name=name;p.x=pose.Pos().X();p.y=pose.Pos().Y();frame.positions.push_back(p);
    }
    frame.events.swap(State().events); publisher_->publish(frame);
  }
  gazebo::physics::WorldPtr world_; gazebo::event::ConnectionPtr end_;
  std::shared_ptr<Domain> domain_; rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Frame>::SharedPtr publisher_; rclcpp::Service<Control>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_; std::vector<std::string> names_;
  uint64_t token_{0}; std::string phase_{"UNCONFIGURED"},error_;
  std::shared_ptr<bool> alive_{std::make_shared<bool>(true)};
};
GZ_REGISTER_WORLD_PLUGIN(World)
}
