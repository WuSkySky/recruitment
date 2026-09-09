#include <gtest/gtest.h>
#include <sdf/parser.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Link.hh>
// White-box test of the private command handler without asynchronous transport.
// This target compiles the implementation itself, not the shared plugin library.
#include "../plugins/light_bar_controller/LightBarController.cc"

TEST(LightBarController, UpdatesRenderedAndPersistentMaterial)
{
  EntityComponentManager ecm;
  const auto entity = ecm.CreateEntity();
  LightBarControllerPrivate controller;
  controller.visualEntityInfos.emplace_back(entity);
  for (const int color : {1, 2, 4, 1, 1}) {
    msgs::Int32 command;
    command.set_data(color);
    controller.OnCmd(command);
    controller.UpdateVisualEntities(ecm);
    const auto visual = ecm.Component<components::VisualCmd>(entity);
    const auto material = ecm.Component<components::Material>(entity);
    ASSERT_NE(nullptr, visual);
    ASSERT_NE(nullptr, material);
    EXPECT_EQ(entity, visual->Data().id());
    EXPECT_EQ(GetMaterial(color).Diffuse(), material->Data().Diffuse());
    EXPECT_EQ(GetMaterial(color).Diffuse(),
      convert<sdf::Material>(visual->Data().material()).Diffuse());
  }
}

TEST(LightBarController, InvalidTransportColorDoesNotChangeTarget)
{
  LightBarControllerPrivate controller;
  msgs::Int32 command;
  command.set_data(2);
  controller.OnCmd(command);
  controller.change = false;
  for (const int invalid : {-1, 5, 99}) {
    command.set_data(invalid);
    controller.OnCmd(command);
    EXPECT_EQ(2, controller.targetState);
    EXPECT_FALSE(controller.change);
  }
}

TEST(LightBarController, AppliesInitialColorWhilePaused)
{
  EntityComponentManager ecm;
  const auto model = ecm.CreateEntity();
  ecm.CreateComponent(model, components::Model());
  ecm.CreateComponent(model, components::Name("test_paused_armor"));
  const auto link = ecm.CreateEntity();
  ecm.CreateComponent(link, components::Link());
  ecm.CreateComponent(link, components::Name("armor"));
  ecm.CreateComponent(link, components::ParentEntity(model));
  const auto visual = ecm.CreateEntity();
  ecm.CreateComponent(visual, components::Visual());
  ecm.CreateComponent(visual, components::Name("light_bar_visual"));
  ecm.CreateComponent(visual, components::ParentEntity(link));
  auto config = std::make_shared<sdf::SDF>();
  sdf::init(config);
  ASSERT_TRUE(sdf::readString(
    "<sdf version='1.8'><model name='test'><link name='armor'/>"
    "<plugin name='test' filename='test'><initial_color>blue</initial_color>"
    "<link_visual>armor/light_bar_visual</link_visual></plugin></model></sdf>", config));
  LightBarController controller;
  EventManager events;
  controller.Configure(model, config->Root()->GetElement("model")->GetElement("plugin"), ecm, events);
  UpdateInfo info;
  info.paused = true;
  controller.PreUpdate(info, ecm);
  const auto cmd = ecm.Component<components::VisualCmd>(visual);
  ASSERT_NE(nullptr, cmd);
  EXPECT_EQ(GetMaterial(2).Diffuse(), convert<sdf::Material>(cmd->Data().material()).Diffuse());
}
