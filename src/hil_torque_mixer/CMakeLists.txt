cmake_minimum_required(VERSION 3.14)
project(hil_torque_mixer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(hil_torque_mixer_node src/hil_torque_mixer_node.cpp)

ament_target_dependencies(hil_torque_mixer_node
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS hil_torque_mixer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
