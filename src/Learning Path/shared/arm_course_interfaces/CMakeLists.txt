cmake_minimum_required(VERSION 3.16)
project(arm_course_interfaces)
set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "ROS system Python" FORCE)
set(Python_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "ROS system Python" FORCE)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Object.msg msg/Objects.msg action/Task.action action/MoveXYZ.action
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
