cmake_minimum_required(VERSION 3.8)
project(recruitment_sim_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

add_library(RecruitmentSimMecanumDrive SHARED
  plugins/mecanum_drive2/MecanumDrive2.cc
)
target_link_libraries(RecruitmentSimMecanumDrive
  ignition-gazebo6::ignition-gazebo6
)

add_library(RecruitmentSimProjectileShooter SHARED
  plugins/projectile_shooter/ProjectileShooter.cc
)
target_link_libraries(RecruitmentSimProjectileShooter
  ignition-gazebo6::ignition-gazebo6
)

add_library(RecruitmentSimLightBarController SHARED
  plugins/light_bar_controller/LightBarController.cc
)
target_link_libraries(RecruitmentSimLightBarController
  ignition-gazebo6::ignition-gazebo6
)

install(TARGETS
  RecruitmentSimMecanumDrive
  RecruitmentSimProjectileShooter
  RecruitmentSimLightBarController
  LIBRARY DESTINATION plugins
)

install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_package()
