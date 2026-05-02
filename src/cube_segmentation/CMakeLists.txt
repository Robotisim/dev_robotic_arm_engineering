cmake_minimum_required(VERSION 3.8)
project(cube_segmentation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/color_segmentation_node
    scripts/moveit_pose_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
