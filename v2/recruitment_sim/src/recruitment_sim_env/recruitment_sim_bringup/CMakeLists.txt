cmake_minimum_required(VERSION 3.8)
project(recruitment_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(recruitment_sim_interfaces REQUIRED)
add_executable(team_topic_bridge src/team_topic_bridge.cpp)
target_compile_features(team_topic_bridge PUBLIC cxx_std_17)
ament_target_dependencies(team_topic_bridge rclcpp geometry_msgs std_msgs
  rosgraph_msgs recruitment_sim_interfaces)
install(TARGETS team_topic_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
