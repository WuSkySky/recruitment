cmake_minimum_required(VERSION 3.8)
project(recruitment_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  resource
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
