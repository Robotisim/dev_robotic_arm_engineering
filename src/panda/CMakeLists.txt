cmake_minimum_required(VERSION 3.8)
project(panda)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/example_position
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
