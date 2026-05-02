cmake_minimum_required(VERSION 3.8)
project(motion)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/motion_01_point_to_point
    scripts/motion_02_straight_line
    scripts/motion_03_circle_bump
    scripts/motion_04_rectangle_shape
    scripts/motion_05_workspace_reject
    scripts/motion_06_moveit_pose_goal
    scripts/motion_07_moveit_arm_hand_pose_goal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
