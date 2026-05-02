cmake_minimum_required(VERSION 3.8)
project(robotic_arm_sim)

find_package(ament_cmake REQUIRED)

ament_environment_hooks(
  "cmake/environment_hooks/gz_sim_resource_path.bash"
  "cmake/environment_hooks/sdf_path.bash"
)

install(
  DIRECTORY cmake models panda worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
