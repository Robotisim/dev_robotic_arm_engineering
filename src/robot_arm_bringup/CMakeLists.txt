cmake_minimum_required(VERSION 3.8)
project(robot_arm_bringup)

find_package(ament_cmake REQUIRED)


install(DIRECTORY
  config
  launch
  meshes
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
