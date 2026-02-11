cmake_minimum_required(VERSION 3.8)
project(panda)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

ament_environment_hooks(
  "cmake/environment_hooks/gz_sim_resource_path.bash"
  "cmake/environment_hooks/sdf_path.bash"
)

add_executable(example_position src/example_position.cpp)
ament_target_dependencies(example_position
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

install(
  TARGETS
    example_position
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    cmake
    config
    launch
    meshes
    models
    panda
    rviz
    scripts
    src
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
