cmake_minimum_required(VERSION 3.8)
project(recruitment_sim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/MatchStatus.msg"
  "msg/PlayerInput.msg"
  "srv/ControlMatch.srv"
  "srv/InitializeModule.srv"
  "srv/ResetRobot.srv"
  "srv/SetRobotEnabled.srv"
  "srv/SetLightColor.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
