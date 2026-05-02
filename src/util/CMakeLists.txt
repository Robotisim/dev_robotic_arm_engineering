cmake_minimum_required(VERSION 3.8)
project(util)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/pcd_snapshot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
