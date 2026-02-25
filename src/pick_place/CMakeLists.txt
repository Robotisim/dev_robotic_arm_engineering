cmake_minimum_required(VERSION 3.8)
project(pick_place)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
	include
	${OpenCV_INCLUDE_DIRS}
)

# Dependencies list for target linking
set(dependencies
	rclcpp
	sensor_msgs
	geometry_msgs
	cv_bridge
	image_geometry
	tf2_ros
	tf2_geometry_msgs
	moveit_ros_planning_interface
	OpenCV
)

# Build object_detector_node executable (includes all source files)
add_executable(object_detector_node
	src/object_detector.cpp
	src/object_detector_main.cpp
)
ament_target_dependencies(object_detector_node ${dependencies})

# Install executable
install(TARGETS
	object_detector_node
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
	launch
	config
	DESTINATION share/${PROJECT_NAME}/
)

# Install include directory
install(DIRECTORY include/
	DESTINATION include
)

# Testing
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
