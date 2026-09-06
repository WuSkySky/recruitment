cmake_minimum_required(VERSION 3.8)
project(recruitment_sim_robot_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(recruitment_sim_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gz_module/gz_chassis_actuator.cpp
  src/gz_module/gz_shoot_actuator.cpp
  src/gz_module/gz_odometry.cpp
  src/gz_module/gz_light_bar_cmd.cpp
  src/chassis_controller.cpp
  src/gimbal_interface.cpp
  src/shooter_controller.cpp
  src/odometry_publisher.cpp
  src/robot_base_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  recruitment_sim_interfaces
  std_msgs
)
target_link_libraries(${PROJECT_NAME}
  ignition-msgs8::core
  ignition-transport11::core
)

add_executable(robot_base src/robot_base_main.cpp)
target_link_libraries(robot_base ${PROJECT_NAME})
rclcpp_components_register_nodes(${PROJECT_NAME} "recruitment_sim_robot_base::RobotBaseNode")

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS robot_base DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/test_light_color.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs nav_msgs rclcpp rclcpp_components
  recruitment_sim_interfaces std_msgs
)
ament_export_dependencies(ignition-msgs8 ignition-transport11)
ament_package()
