#include "pick_place/object_detector.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<pick_place::ObjectDetector>();

	RCLCPP_INFO(node->get_logger(), "Starting object_detector_node...");

	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
