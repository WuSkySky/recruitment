cmake_minimum_required(VERSION 3.8)
project(recruitment_sim_referee_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(recruitment_sim_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_engine src/referee_engine.cpp)
target_include_directories(${PROJECT_NAME}_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(referee_system src/referee_system_node.cpp)
target_include_directories(referee_system PRIVATE include)
target_link_libraries(referee_system
  ${PROJECT_NAME}_engine
  ignition-msgs8::core
  ignition-transport11::core
)
ament_target_dependencies(referee_system
  rclcpp
  recruitment_sim_interfaces
  std_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_engine
  EXPORT ${PROJECT_NAME}_engine
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS referee_system DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_referee_engine test/test_referee_engine.cpp)
  target_link_libraries(test_referee_engine ${PROJECT_NAME}_engine)
  target_include_directories(test_referee_engine PRIVATE include)
endif()

ament_export_targets(${PROJECT_NAME}_engine HAS_LIBRARY_TARGET)
ament_package()
