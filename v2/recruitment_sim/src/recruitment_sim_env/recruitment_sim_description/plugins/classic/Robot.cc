// Copyright 2021 RoboMaster-OSS
// Copyright 2026 Recruitment Simulation Maintainers
// Licensed under the Apache License, Version 2.0.
// Classic adaptation of the original mecanum, gimbal, shooter and light controllers.
#include "Runtime.hh"
#include "../projectile_shooter/DirectionNoise.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <recruitment_sim_robot_base/enable_state.hpp>
#include <recruitment_sim_robot_base/gaussian_noise.hpp>
#include <recruitment_sim_robot_base/incremental_odometry.hpp>
#include <recruitment_sim_interfaces/srv/set_robot_enabled.hpp>
#include <recruitment_sim_interfaces/srv/set_light_color.hpp>
#include <recruitment_sim_interfaces/srv/reset_robot.hpp>
#include <recruitment_sim_interfaces/srv/initialize_module.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ignition/math/PID.hh>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace recruitment_sim {
namespace base = recruitment_sim_robot_base;
using Enable = recruitment_sim_interfaces::srv::SetRobotEnabled;
using Light = recruitment_sim_interfaces::srv::SetLightColor;
using ResetService = recruitment_sim_interfaces::srv::ResetRobot;
using Initialize = recruitment_sim_interfaces::srv::InitializeModule;
class Robot : public gazebo::ModelPlugin {
  using Float = std_msgs::msg::Float64;
  using Odom = nav_msgs::msg::Odometry;
  struct Projectile {
    std::string name; uint64_t id; double requested; ignition::math::Vector3d velocity;
    bool spawned{false}, removed{false};
  };
public:
  ~Robot() override {
    update_.reset(); end_.reset();
    {std::lock_guard<std::recursive_mutex> lock(State().mutex); *alive_ = false; State().robots.erase(name_);}
    if(domain_) domain_->Remove(node_);
    if(sensor_domain_) sensor_domain_->Remove(sensor_node_);
  }
  template<class F> typename Signature<decltype(&F::operator())>::type Safe(F f) {
    auto alive = alive_;
    return [alive, f](auto... args) {
      std::lock_guard<std::recursive_mutex> lock(State().mutex);
      if(*alive) f(args...);
    };
  }
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model; world_ = model->GetWorld(); name_ = model->GetName();
    chassis_ = model->GetLink("chassis"); muzzle_ = model->GetLink("speed_monitor");
    yaw_ = model->GetJoint("gimbal_yaw_joint"); pitch_ = model->GetJoint("gimbal_pitch_joint");
    if(!chassis_ || !muzzle_ || !yaw_ || !pitch_) throw std::runtime_error("missing required robot links/joints");
    // Classic/ODE 下 Joint::SetVelocity 是直接改写子链接速度（SetVelocityMaximal），
    // 不经过关节电机，既压不住重力、也无法稳定跟随指令。改用 ODE 电机：
    // 这里给定力矩上限，Update() 里用 "vel" 参数下发目标关节速度。
    // Fortress 版本由 JointController 的 PID 提供力矩，不需要这一步。
    yaw_->SetParam("fmax", 0, 20.0);
    pitch_->SetParam("fmax", 0, 20.0);
    ns_ = sdf->Get<std::string>("namespace");
    domain_ = GetDomain(InternalDomain()); node_ = domain_->Node("robot_base", ns_);
    sensor_domain_ = GetDomain(sdf->Get<unsigned int>("sensor_domain"));
    sensor_node_ = sensor_domain_->Node("chassis_sensor", ns_);
    use_odom_ = sdf->Get<bool>("use_odometry");
    direction_.Configure(sdf->Get<double>("yaw_angle_variance"), sdf->Get<double>("pitch_angle_variance"));
    auto config = sdf->GetElement("noise");
    for(auto el = config->GetFirstElement(); el; el = el->GetNextElement())
      noise_.emplace(el->GetName(), base::GaussianNoise(el->Get<double>()));
    color_ = sdf->Get<unsigned int>("initial_color");
    transport_.reset(new gazebo::transport::Node); transport_->Init(world_->Name());
    visual_pub_ = transport_->Advertise<gazebo::msgs::Visual>("~/visual");
    const auto qos = rclcpp::QoS(10);
    chassis_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_chassis_vel", qos,
      Safe([this](geometry_msgs::msg::Twist::SharedPtr m) {
        if(enabled_.chassis_enabled() && std::isfinite(m->linear.x) && std::isfinite(m->linear.y) && std::isfinite(m->angular.z)) command_ = *m;
      }));
    yaw_sub_ = node_->create_subscription<Float>("cmd_yaw_vel", qos, Safe([this](Float::SharedPtr m) {
      if(enabled_.gimbal_enabled() && std::isfinite(m->data)) yaw_cmd_ = m->data;
    }));
    pitch_sub_ = node_->create_subscription<Float>("cmd_pitch_vel", qos, Safe([this](Float::SharedPtr m) {
      if(enabled_.gimbal_enabled() && std::isfinite(m->data)) pitch_cmd_ = m->data;
    }));
    shoot_sub_ = node_->create_subscription<std_msgs::msg::Bool>("cmd_shoot", qos,
      Safe([this](std_msgs::msg::Bool::SharedPtr m) {if(enabled_.shooter_enabled()) shooting_ = m->data;}));
    for(auto suffix : {"yaw_vel", "pitch_vel", "yaw_angle", "pitch_angle"})
      feedback_[suffix] = node_->create_publisher<Float>(std::string("feedback_")+suffix, qos);
    truth_pub_ = sensor_node_->create_publisher<Odom>("chassis_odometry", qos);
    if(use_odom_) odom_pub_ = node_->create_publisher<Odom>("robot_base/odom", qos);
    std::string prefix = "/referee_system/"+name_;
    enable_srv_ = node_->create_service<Enable>(prefix+"/set_enabled", Safe(
      [this](Enable::Request::SharedPtr req, Enable::Response::SharedPtr res) {
        res->success = req->round_id >= epoch_ && enabled_.set(req->target, req->enabled);
        if(res->success) {epoch_ = req->round_id; StopDisabled();}
        res->message = res->success ? "applied" : "invalid target or stale generation";
      }));
    reset_srv_ = node_->create_service<ResetService>(prefix+"/reset", Safe(
      [this](ResetService::Request::SharedPtr req, ResetService::Response::SharedPtr res) {
        res->success = req->round_id >= epoch_ && req->color <= 4;
        if(res->success) {
          epoch_ = req->round_id; enabled_.reset(req->enabled); ClearCommands();
          color_ = req->color; UpdateLights();
        }
        res->message = res->success ? "applied" : "invalid color or stale generation";
      }));
    init_srv_ = node_->create_service<Initialize>(prefix+"/initialize_odometry", Safe(
      [this](Initialize::Request::SharedPtr req, Initialize::Response::SharedPtr res) {
        res->success = req->round_id >= init_epoch_;
        if(res->success && (!initialized_request_ || req->round_id > init_epoch_)) {
          init_epoch_ = req->round_id; initialized_request_ = true; odom_initialized_ = false;
        }
        res->message = res->success ? "odometry cleared; next sample starts at zero" : "stale generation";
      }));
    light_srv_ = node_->create_service<Light>("robot_base/set_light_color", Safe(
      [this](Light::Request::SharedPtr req, Light::Response::SharedPtr res) {
        res->success = req->color <= 4; if(res->success) {color_ = req->color; UpdateLights();}
      }));
    x_pid_.Init(100,0,0,0,0,100,-100,0); y_pid_.Init(500,0,0,0,0,200,-200,0); w_pid_.Init(200,0,0,0,0,100,-100,0);
    {std::lock_guard<std::recursive_mutex> lock(State().mutex);
      State().robots[name_] = {model_, model_->WorldPose(), [this] {ResetPhysics();}, [this] {Contacts();}};
    }
    world_->Physics()->GetContactManager()->SetNeverDropContacts(true);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(Safe([this](const gazebo::common::UpdateInfo &) {Update();}));
    // Material updates also run while paused. No physical state is modified here.
    light_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), Safe([this] {UpdateLights();}));
  }
private:
  double Noise(const std::string & key, double value) {return noise_.at(key).apply(value);}
  void ClearCommands() {command_ = geometry_msgs::msg::Twist(); yaw_cmd_ = pitch_cmd_ = 0; shooting_ = false; noisy_command_ = geometry_msgs::msg::Twist(); noisy_yaw_ = noisy_pitch_ = 0;}
  void StopDisabled() {
    if(!enabled_.chassis_enabled()) {command_ = geometry_msgs::msg::Twist(); noisy_command_ = geometry_msgs::msg::Twist();}
    if(!enabled_.gimbal_enabled()) {yaw_cmd_ = pitch_cmd_ = noisy_yaw_ = noisy_pitch_ = 0;}
    if(!enabled_.shooter_enabled()) shooting_ = false;
  }
  void ResetPhysics() {
    ClearCommands(); x_pid_.Reset(); y_pid_.Reset(); w_pid_.Reset();
    for(const auto & joint : model_->GetJoints()) {if(joint->DOF()>0) {joint->SetPosition(0,0); joint->SetVelocity(0,0);}}
    model_->SetLinearVel({0,0,0}); model_->SetAngularVel({0,0,0});
    // 关节已回到 0，位置保持的目标与积分也必须归零，否则云台会自己转回去。
    yaw_target_ = pitch_target_ = yaw_integral_ = pitch_integral_ = 0;
    projectiles_.clear(); shot_id_ = 0; last_shot_ = -1; odom_initialized_ = false;
    last_control_ = last_feedback_ = last_odom_ = last_truth_pub_ = -1;
  }
  void UpdateLights() {
    static const ignition::math::Color colors[] = {{0,0,0,1},{1,0,0,1},{0,0,1,1},{1,1,0,1},{1,1,1,1}};
    for(const auto & link : model_->GetLinks()) {
      auto sdf = link->GetSDF(); if(!sdf->HasElement("visual")) continue;
      for(auto visual = sdf->GetElement("visual"); visual; visual = visual->GetNextElement("visual")) {
        if(visual->Get<std::string>("name") != "light_bar_visual") continue;
        gazebo::msgs::Visual msg;
        msg.set_name(link->GetScopedName()+"::light_bar_visual"); msg.set_parent_name(link->GetScopedName());
        auto material = msg.mutable_material();
        gazebo::msgs::Set(material->mutable_ambient(), colors[color_]);
        gazebo::msgs::Set(material->mutable_diffuse(), colors[color_]);
        gazebo::msgs::Set(material->mutable_emissive(), colors[color_]);
        visual_pub_->Publish(msg);
      }
    }
  }
  void Update() {
    const double now = world_->SimTime().Double();
    if(world_->IsPaused()) return;
    if(now-last_control_ >= .01 || last_control_<0) {
      last_control_ = now;
      if(enabled_.chassis_enabled()) {
        noisy_command_.linear.x = Noise("actuator_chassis_x_velocity_variance",command_.linear.x);
        noisy_command_.linear.y = Noise("actuator_chassis_y_velocity_variance",command_.linear.y);
        noisy_command_.angular.z = Noise("actuator_chassis_yaw_velocity_variance",command_.angular.z);
      }
      if(enabled_.gimbal_enabled()) {
        noisy_yaw_ = Noise("actuator_gimbal_yaw_velocity_variance",yaw_cmd_);
        noisy_pitch_ = Noise("actuator_gimbal_pitch_velocity_variance",pitch_cmd_);
      }
    }
    auto v = chassis_->RelativeLinearVel(); auto w = chassis_->RelativeAngularVel();
    const auto dt = std::chrono::duration<double>(world_->Physics()->GetMaxStepSize());
    chassis_->AddRelativeForce({x_pid_.Update(v.X()-noisy_command_.linear.x, dt),y_pid_.Update(v.Y()-noisy_command_.linear.y,dt),0});
    chassis_->AddRelativeTorque({0,0,w_pid_.Update(w.Z()-noisy_command_.angular.z,dt)});
    // 速度指令积分成目标角，再叠加位置 PI：对应 Fortress 版本 JointController 的
    // p_gain/i_gain。Classic 的 ODE 速度电机是软约束，纯速度指令在重力力矩下会有
    // 稳态爬行（实测约 0.002 rad/s），位置项把它压住，积分项消除残差。
    const double step = dt.count();
    yaw_target_ += noisy_yaw_ * step;
    pitch_target_ += noisy_pitch_ * step;
    const double yaw_error = yaw_target_ - yaw_->Position(0);
    const double pitch_error = pitch_target_ - pitch_->Position(0);
    yaw_integral_ = std::clamp(yaw_integral_ + yaw_error * step, -.5, .5);
    pitch_integral_ = std::clamp(pitch_integral_ + pitch_error * step, -.5, .5);
    yaw_->SetParam("vel",0,std::clamp(noisy_yaw_ + 8.0*yaw_error + 2.0*yaw_integral_,-10.0,10.0));
    pitch_->SetParam("vel",0,std::clamp(noisy_pitch_ + 8.0*pitch_error + 2.0*pitch_integral_,-10.0,10.0));
    SpawnPending(now);
    // Gazebo Classic 只在每 200 ms（墙钟）处理一次模型插入（World::processMsgsPeriod），
    // 所以 50 ms 连发时多枚弹丸会在同一仿真时刻、同一炮口位置被创建并互相碰撞销毁。
    // 原始 Fortress 实现用“上一枚完成初始化后才发下一枚”规避，这里恢复该约束。
    const bool launcher_ready = projectiles_.empty() || projectiles_.back().spawned;
    if(shooting_ && enabled_.shooter_enabled() && launcher_ready && (last_shot_<0 || now-last_shot_ >= .05-1e-9)) {
      Shoot(now); last_shot_ = now;
    }
    if(last_feedback_<0 || now-last_feedback_ >= .01-1e-9) {
      last_feedback_=now;
      PublishFeedback("yaw_vel",Noise("sensor_gimbal_yaw_velocity_variance",yaw_->GetVelocity(0)));
      PublishFeedback("pitch_vel",Noise("sensor_gimbal_pitch_velocity_variance",pitch_->GetVelocity(0)));
      PublishFeedback("yaw_angle",base::normalize_angle(Noise("sensor_gimbal_yaw_angle_variance",yaw_->Position(0))));
      PublishFeedback("pitch_angle",base::normalize_angle(Noise("sensor_gimbal_pitch_angle_variance",pitch_->Position(0))));
    }
    Odometry();
  }
  void PublishFeedback(const std::string & key, double value) {Float msg; msg.data=value; feedback_.at(key)->publish(msg);}
  void Odometry() {
    auto pose=chassis_->WorldPose(); auto v=chassis_->RelativeLinearVel(); auto w=chassis_->RelativeAngularVel();
    const double now=world_->SimTime().Double();
    // 真值里程计按 100 Hz 发布。物理步是 1 kHz，逐步发布会产生约 4 倍于其它话题的
    // 消息量（4 台车约 2700 msg/s），而消费端（选手/Web/RViz）远不需要这个速率。
    if(last_truth_pub_<0 || now-last_truth_pub_ >= .01-1e-9) {
      last_truth_pub_=now;
      Odom msg; msg.header.stamp=rclcpp::Time(Nanoseconds(world_->SimTime()));
      msg.header.frame_id=name_+"/odom"; msg.child_frame_id=name_+"/chassis";
      msg.pose.pose.position.x=pose.Pos().X(); msg.pose.pose.position.y=pose.Pos().Y(); msg.pose.pose.position.z=0;
      auto & q=msg.pose.pose.orientation; q.x=q.y=0; q.z=std::sin(pose.Rot().Yaw()/2); q.w=std::cos(pose.Rot().Yaw()/2);
      msg.twist.twist.linear.x=v.X(); msg.twist.twist.linear.y=v.Y(); msg.twist.twist.angular.z=w.Z();
      truth_pub_->publish(msg);
    }
    if(!use_odom_ || (last_odom_>=0 && now-last_odom_<1.0/30)) return;
    last_odom_=now;
    Odom msg; msg.header.stamp=rclcpp::Time(Nanoseconds(world_->SimTime()));
    base::PlanarPose truth{pose.Pos().X(),pose.Pos().Y(),pose.Rot().Yaw()};
    if(!odom_initialized_) {integrated_={}; odom_initialized_=true;}
    else integrated_=base::integrate_odometry_increment(last_truth_,truth,integrated_,
      Noise("sensor_chassis_x_position_increment_variance",0),Noise("sensor_chassis_y_position_increment_variance",0),Noise("sensor_chassis_yaw_increment_variance",0));
    last_truth_=truth;
    msg.header.frame_id="odom"; msg.child_frame_id="base_link";
    msg.pose.pose.position.x=integrated_.x; msg.pose.pose.position.y=integrated_.y;
    auto & q=msg.pose.pose.orientation; q.x=q.y=0; q.z=std::sin(integrated_.yaw/2); q.w=std::cos(integrated_.yaw/2);
    msg.twist.twist.linear.x=Noise("sensor_chassis_x_velocity_variance",v.X());
    msg.twist.twist.linear.y=Noise("sensor_chassis_y_velocity_variance",v.Y());
    msg.twist.twist.angular.z=Noise("sensor_chassis_yaw_velocity_variance",w.Z());
    odom_pub_->publish(msg);
  }
  void Shoot(double now) {
    const auto pose=muzzle_->WorldPose()*ignition::math::Pose3d(.15,0,0,0,0,0);
    Projectile p{name_+"_projectile_"+std::to_string(State().round)+"_"+std::to_string(shot_id_),shot_id_++,now,pose.Rot().RotateVector(direction_.Sample(18))};
    std::ostringstream xml; xml<<std::setprecision(17)<<"<sdf version='1.6'><model name='"<<p.name<<"'><pose>"<<pose<<"</pose><link name='link'><inertial><mass>0.0032</mass><inertia><ixx>9.03168e-8</ixx><iyy>9.03168e-8</iyy><izz>9.03168e-8</izz></inertia></inertial><collision name='collision'><geometry><sphere><radius>0.0084</radius></sphere></geometry></collision><visual name='visual'><geometry><sphere><radius>0.0084</radius></sphere></geometry><material><ambient>0 0.6 0 1</ambient><diffuse>0 0.6 0 1</diffuse><emissive>0 0.6 0 1</emissive></material></visual></link></model></sdf>";
    world_->InsertModelString(xml.str()); State().projectiles.push_back(p.name); projectiles_.push_back(p);
  }
  void SpawnPending(double now) {
    for(auto & p:projectiles_) {
      if(p.removed) {world_->RemoveModel(p.name); continue;}
      auto model=world_->ModelByName(p.name);
      if(!p.spawned && model) {
        model->SetLinearVel(p.velocity); p.spawned=true;
        Event event; event.kind=Event::SHOT; event.shooter=name_; event.projectile_id=p.id; State().events.push_back(event);
      }
      if(now-p.requested>4 && model) {world_->RemoveModel(model); p.removed=true;}
    }
    projectiles_.erase(std::remove_if(projectiles_.begin(),projectiles_.end(),[](const auto & p){return p.removed;}),projectiles_.end());
  }
  void Contacts() {
    if(world_->IsPaused()) return;
    auto manager=world_->Physics()->GetContactManager();
    for(unsigned int i=0;i<manager->GetContactCount();++i) {
      auto c=manager->GetContacts()[i]; if(!c->collision1 || !c->collision2 || !c->collision1->GetLink() || !c->collision2->GetLink()) continue;
      for(auto & p:projectiles_) {
        if(!p.spawned || p.removed) continue;
        gazebo::physics::Collision * target=nullptr;
        if(c->collision1->GetModel()->GetName()==p.name) target=c->collision2;
        else if(c->collision2->GetModel()->GetName()==p.name) target=c->collision1;
        if(!target) continue;
        Event event; event.kind=Event::HIT; event.shooter=name_; event.projectile_id=p.id;
        event.target=target->GetModel()->GetName(); event.target_link=target->GetLink()->GetName(); event.target_collision=target->GetName();
        State().events.push_back(event); p.removed=true;
      }
    }
  }
  std::shared_ptr<bool> alive_{std::make_shared<bool>(true)};
  gazebo::physics::ModelPtr model_; gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr chassis_,muzzle_; gazebo::physics::JointPtr yaw_,pitch_;
  gazebo::event::ConnectionPtr update_,end_;
  std::shared_ptr<Domain> domain_,sensor_domain_; rclcpp::Node::SharedPtr node_,sensor_node_;
  std::string name_,ns_; base::EnableState enabled_; DirectionNoise direction_;
  std::map<std::string,base::GaussianNoise> noise_;
  ignition::math::PID x_pid_,y_pid_,w_pid_;
  geometry_msgs::msg::Twist command_,noisy_command_;
  double yaw_cmd_{0},pitch_cmd_{0},noisy_yaw_{0},noisy_pitch_{0}; bool shooting_{false},use_odom_{false};
  double yaw_target_{0},pitch_target_{0},yaw_integral_{0},pitch_integral_{0};
  uint64_t epoch_{0},init_epoch_{0},shot_id_{0}; unsigned int color_{0};
  bool initialized_request_{false},odom_initialized_{false}; base::PlanarPose last_truth_,integrated_;
  double last_control_{-1},last_feedback_{-1},last_odom_{-1},last_shot_{-1},last_truth_pub_{-1};
  std::vector<Projectile> projectiles_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_sub_;
  rclcpp::Subscription<Float>::SharedPtr yaw_sub_,pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_sub_;
  std::map<std::string,rclcpp::Publisher<Float>::SharedPtr> feedback_;
  rclcpp::Publisher<Odom>::SharedPtr truth_pub_,odom_pub_;
  rclcpp::Service<Enable>::SharedPtr enable_srv_; rclcpp::Service<ResetService>::SharedPtr reset_srv_;
  rclcpp::Service<Initialize>::SharedPtr init_srv_; rclcpp::Service<Light>::SharedPtr light_srv_;
  gazebo::transport::NodePtr transport_; gazebo::transport::PublisherPtr visual_pub_;
  rclcpp::TimerBase::SharedPtr light_timer_;
};
GZ_REGISTER_MODEL_PLUGIN(Robot)
}
