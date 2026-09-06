cmake_minimum_required(VERSION 3.8)
project(recruitment_sim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisCmd.msg"
  "msg/Gimbal.msg"
  "msg/GimbalCmd.msg"
  "msg/ShootCmd.msg"
  "srv/SetLightColor.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
