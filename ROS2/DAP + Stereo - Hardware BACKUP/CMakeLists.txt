cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go_to_goal_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()