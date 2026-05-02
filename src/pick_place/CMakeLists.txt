cmake_minimum_required(VERSION 3.8)
project(pick_place)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/object_detector_node
    scripts/object_detector_pcl_node
    scripts/pick_place_executor_node
    scripts/pick_place_multi_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
