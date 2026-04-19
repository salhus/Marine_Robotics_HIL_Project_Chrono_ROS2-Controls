cmake_minimum_required(VERSION 3.8)
project(hil_odrive_ros2_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
