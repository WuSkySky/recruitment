// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mutex>
#include <list>
#include <sstream>
#include <iomanip>
#include "../referee_simulation/EventBus.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "ProjectileShooter.hh"
#include "DirectionNoise.hh"
#include <sdf/Model.hh>
#include <sdf/Root.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

struct ProjectileInfo {
    Entity entity;
    std::string name;
    unsigned int id;
    std::chrono::steady_clock::duration spawnTime;
    bool isInit;
    bool remove{false};
    ProjectileInfo(Entity _entity, std::string _name, unsigned int _id,
                    std::chrono::steady_clock::duration _time)
        : entity(_entity)
        , name(_name)
        , id(_id)
        , spawnTime(_time)
        , isInit(false)
    {
    }
};

class ignition::gazebo::systems::ProjectileShooterPrivate {
public:
    void OnCmd(const ignition::msgs::Boolean& _msg);
    void PreUpdate(const ignition::gazebo::UpdateInfo& _info, 
                    ignition::gazebo::EntityComponentManager& _ecm);
    void PublishShotInfo(
        const ignition::gazebo::UpdateInfo& _info, const ProjectileInfo& _projectile);
    void PublishHitInfo(
        const ignition::gazebo::UpdateInfo& _info,
        ignition::gazebo::EntityComponentManager& _ecm,
        const ProjectileInfo& _projectile,
        Entity _targetCollision);
public:
    // node and tool
    transport::Node node;
    transport::Node::Publisher shotInfoPub;
    transport::Node::Publisher hitInfoPub;
    std::unique_ptr<SdfEntityCreator> creator { nullptr };
    bool initialized { false };
    // entity: world,model, shooter link
    Entity world { kNullEntity };
    Entity model { kNullEntity };
    Entity shooterLink { kNullEntity };
    std::string modelName;
    std::string shooterName {"default_shooter"};
    math::Pose3d shooterOffset;
    // parameters of shooter
    double shootVel { 18 };
    recruitment_sim::DirectionNoise directionNoise;
    double shootPeriodMS { 50 };
    // projectile
    sdf::Model projectileSdfModel;
    unsigned int projectileId{ 0 };
    bool shootEnabled { false };
    std::mutex shootStateMutex;
    std::list<ProjectileInfo> spawnedProjectiles;
    std::chrono::steady_clock::duration lastSpawnTime;
    uint64_t roundId{0};
};

/******************implementation for ProjectileShooter************************/
ProjectileShooter::ProjectileShooter()
    : dataPtr(std::make_unique<ProjectileShooterPrivate>())
{
}

void ProjectileShooter::Configure(const Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    EntityComponentManager& _ecm,
    EventManager& _eventMgr)
{
    this->dataPtr->model = _entity;
    auto modelWrapper = Model(_entity);
    if (!modelWrapper.Valid(_ecm)) {
        ignerr << "ProjectileShooter plugin should be attached to a model entity. \
                     Failed to initialize." << std::endl;
        return;
    }
    this->dataPtr->modelName = modelWrapper.Name(_ecm);
    // Get params from SDF
    auto linkName= _sdf->Get<std::string>("shooter_link");
    this->dataPtr->shooterLink = modelWrapper.LinkByName(_ecm, linkName);
    if (this->dataPtr->shooterLink == kNullEntity) {
        ignerr << "shooter link with name[" << linkName << "] not found. " << std::endl;
        return;
    }
    if (_sdf->HasElement("shooter_name")) {
        this->dataPtr->shooterName = _sdf->Get<std::string>("shooter_name");
    }
    if (_sdf->HasElement("shooter_offset")) {
        this->dataPtr->shooterOffset = _sdf->Get<math::Pose3d>("shooter_offset");
    } else{
        ignerr << "The tag <shooter_offset> is not found. " << std::endl;
        return;  
    }
    if (_sdf->HasElement("projectile_velocity")) {
        this->dataPtr->shootVel = _sdf->Get<double>("projectile_velocity");
    }
    std::string projectile_uri;
    try {
        this->dataPtr->directionNoise.Configure(
            _sdf->Get<double>("yaw_angle_variance", 0.0).first,
            _sdf->Get<double>("pitch_angle_variance", 0.0).first);
    } catch (const std::invalid_argument & error) {
        ignerr << "ProjectileShooter: " << error.what() << std::endl;
        return;
    }
    if (_sdf->HasElement("projectile_uri")) {
        projectile_uri = _sdf->Get<std::string>("projectile_uri");
    } else {
        ignerr << "The tag <projectile_uri> is not found." << std::endl;
        return;
    }
    // Load projectile model sdf
    ignition::common::SystemPaths systemPaths;
    sdf::Root projectileSdfRoot;
    sdf::Errors errors = projectileSdfRoot.Load(systemPaths.FindFileURI(projectile_uri));
    if (!errors.empty()) {
        ignerr << "Load Projectile Model:"<<projectile_uri<< std::endl;
        for (const auto& e : errors) {
            ignerr << e.Message() << std::endl;
        }
        return;
    }
    if (projectileSdfRoot.Model() == nullptr) {
        ignerr << "Projectile Model not found" << std::endl;
        return;
    }
    this->dataPtr->projectileSdfModel = *projectileSdfRoot.Model();
    // Subscribe to commands
    std::string shootCmdTopic { this->dataPtr->modelName + "/" + this->dataPtr->shooterName + "/shoot" };
    this->dataPtr->node.Subscribe(shootCmdTopic, &ProjectileShooterPrivate::OnCmd, this->dataPtr.get());
    this->dataPtr->shotInfoPub =
        this->dataPtr->node.Advertise<msgs::StringMsg>("/referee_system/events/shot");
    this->dataPtr->hitInfoPub =
        this->dataPtr->node.Advertise<msgs::StringMsg>("/referee_system/events/hit");
    //creator and world
    this->dataPtr->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventMgr);
    this->dataPtr->world = _ecm.EntityByComponents(components::World());
    this->dataPtr->initialized = true;
    recruitment_sim::RegisterShooter(_entity);
    //debug info
    igndbg << "[" << this->dataPtr->modelName << " ProjectileShooter Info]:" << std::endl;
    igndbg << "Shooter name: " << this->dataPtr->shooterName << std::endl;
    igndbg << "Shooter offset: " << this->dataPtr->shooterOffset << std::endl;
    igndbg << "Projectile name: " << this->dataPtr->projectileSdfModel.Name() << std::endl;
    igndbg << "Shoot CMD Topic: " << shootCmdTopic << std::endl;
}

void ProjectileShooter::PreUpdate(const ignition::gazebo::UpdateInfo& _info,
    ignition::gazebo::EntityComponentManager& _ecm)
{
    this->dataPtr->PreUpdate(_info, _ecm);
}

/******************implementation for ProjectileShooterPrivate******************/

void ProjectileShooterPrivate::OnCmd(const ignition::msgs::Boolean& _msg)
{
    std::lock_guard<std::mutex> lock(this->shootStateMutex);
    this->shootEnabled = _msg.data();
    // ignmsg << "ProjectileShooter OnCmd msg: [" << _msg.data() << "]" << std::endl;
}

void ProjectileShooterPrivate::PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm)
{
    const auto currentRound = recruitment_sim::Round();
    if (currentRound != this->roundId) {
        std::lock_guard<std::mutex> lock(this->shootStateMutex);
        this->roundId = currentRound;
        this->shootEnabled = false;
        this->projectileId = 0;
        this->spawnedProjectiles.clear();
        this->lastSpawnTime = _info.simTime;
    }
    // do nothing if it's paused or not initialized.
    if (_info.paused || !this->initialized) {
        return;
    }
    // Pose of shooter Link
    if (!_ecm.Component<components::WorldPose>(this->shooterLink)) {
        _ecm.CreateComponent(this->shooterLink, components::WorldPose());
    }
    // lock
    std::lock_guard<std::mutex> lock(this->shootStateMutex);
    // spawnFlag CMD
    bool spawnFlag = false;
    double t = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime - this->lastSpawnTime).count();
    if (this->shootEnabled && t >= this->shootPeriodMS) {
        if (this->spawnedProjectiles.empty() || this->spawnedProjectiles.back().isInit) {
            this->lastSpawnTime = _info.simTime;
            spawnFlag = true;
        }
    }    
    // process projectile
    if (spawnFlag) {
        //spawn a projectile
        auto projectileName = this->modelName + "_" + this->shooterName + "_" + std::to_string(this->projectileId);
        this->projectileSdfModel.SetName(projectileName);
            // current state
        auto shooterPose = _ecm.Component<components::WorldPose>(this->shooterLink)->Data();
        this->projectileSdfModel.SetRawPose(shooterPose * this->shooterOffset);
        Entity projectileModel = this->creator->CreateEntities(&this->projectileSdfModel);
        this->creator->SetParent(projectileModel, this->world);
        recruitment_sim::RegisterProjectile(projectileModel);
        //update projectile,set velocity and create ContactSensorData
        // LinearVelocityCmd is in the entity frame; the model already has the muzzle pose.
        const auto tmpVel = this->directionNoise.Sample(this->shootVel);
        _ecm.CreateComponent(projectileModel, components::LinearVelocityCmd({ tmpVel }));
        Entity projectileLink = Model(projectileModel).Links(_ecm)[0];
        Entity projectileCollision = Link(projectileLink).Collisions(_ecm)[0];
        _ecm.CreateComponent(projectileCollision, components::ContactSensorData());
        // update projectile tracking
        ProjectileInfo pInfo(
            projectileModel, projectileName, this->projectileId, _info.simTime);
        this->spawnedProjectiles.push_back(pInfo);
        this->PublishShotInfo(_info, pInfo);
        this->projectileId++;
    } else {
        //try to init the last projectile
        if (!this->spawnedProjectiles.empty()) {
            ProjectileInfo& pInfo = this->spawnedProjectiles.back();
            if (!pInfo.isInit) {
                //cancel velocity
                _ecm.RemoveComponent<components::LinearVelocityCmd>(pInfo.entity);
                pInfo.isInit = true;
            }
        }
    }
    // check
    if (!this->spawnedProjectiles.empty()) {
        // check cantact and time to delete projectiles
        for(auto & pInfo : this->spawnedProjectiles){
            if (!pInfo.isInit) {
                continue;
            }
            Entity link = Model(pInfo.entity).Links(_ecm)[0];
            Entity collision = Link(link).Collisions(_ecm)[0];
            auto contacts = _ecm.Component<components::ContactSensorData>(collision)->Data();
            double t = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime - pInfo.spawnTime).count();
            if (contacts.contact_size() > 0 || t > 4000) {
                if (contacts.contact_size() > 0) {
                    Entity collision1 = contacts.contact(0).collision1().id();
                    Entity collision2 = contacts.contact(0).collision2().id();
                    Entity targetCollision = collision == collision1 ? collision2 : collision1;
                    this->PublishHitInfo(_info, _ecm, pInfo, targetCollision);
                }
                this->creator->RequestRemoveEntity(pInfo.entity);
                recruitment_sim::ForgetProjectile(pInfo.entity);
                pInfo.remove = true;
            }
        }
        // remove from list
        spawnedProjectiles.remove_if([](const ProjectileInfo& pInfo){ return pInfo.remove; });
    }
}

void ProjectileShooterPrivate::PublishShotInfo(
    const ignition::gazebo::UpdateInfo& _info, const ProjectileInfo& _projectile)
{
    std::ostringstream stream;
    stream << this->modelName << ',' << this->shooterName << ',' << _projectile.id << ",17mm";
    msgs::StringMsg msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    msg.set_data(stream.str());
    auto * round = msg.mutable_header()->add_data();
    round->set_key("round_id"); round->add_value(std::to_string(recruitment_sim::Round()));
    this->shotInfoPub.Publish(msg);
    std::ostringstream event;
    event << "S " << std::quoted(this->modelName) << ' ' << _projectile.id;
    recruitment_sim::RecordEvent(event.str());
}

void ProjectileShooterPrivate::PublishHitInfo(
    const ignition::gazebo::UpdateInfo& _info,
    ignition::gazebo::EntityComponentManager& _ecm,
    const ProjectileInfo& _projectile,
    Entity _targetCollision)
{
    const auto collisionName = _ecm.Component<components::Name>(_targetCollision);
    const auto collisionParent = _ecm.Component<components::ParentEntity>(_targetCollision);
    if (!collisionName || !collisionParent) {
        return;
    }

    const Entity targetLink = collisionParent->Data();
    const auto linkName = _ecm.Component<components::Name>(targetLink);
    const auto linkParent = _ecm.Component<components::ParentEntity>(targetLink);
    if (!linkName || !linkParent) {
        return;
    }

    Entity targetModel = linkParent->Data();
    while (targetModel != kNullEntity && !_ecm.Component<components::Model>(targetModel)) {
        const auto parent = _ecm.Component<components::ParentEntity>(targetModel);
        if (!parent) {
            return;
        }
        targetModel = parent->Data();
    }
    const auto modelName = _ecm.Component<components::Name>(targetModel);
    if (!modelName) {
        return;
    }

    std::ostringstream stream;
    stream << this->modelName << ',' << this->shooterName << ',' << _projectile.id << ','
           << modelName->Data() << ',' << linkName->Data() << ',' << collisionName->Data();
    msgs::StringMsg msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    msg.set_data(stream.str());
    auto * round = msg.mutable_header()->add_data();
    round->set_key("round_id"); round->add_value(std::to_string(recruitment_sim::Round()));
    this->hitInfoPub.Publish(msg);
    std::ostringstream event;
    event << "H " << std::quoted(this->modelName) << ' ' << _projectile.id << ' '
          << std::quoted(modelName->Data()) << ' ' << std::quoted(linkName->Data()) << ' '
          << std::quoted(collisionName->Data());
    recruitment_sim::RecordEvent(event.str());
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(ProjectileShooter,
    ignition::gazebo::System,
    ProjectileShooter::ISystemConfigure,
    ProjectileShooter::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ProjectileShooter, "ignition::gazebo::systems::ProjectileShooter")
