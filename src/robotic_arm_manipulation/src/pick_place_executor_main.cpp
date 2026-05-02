#include "pick_place/pick_place_executor.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<pick_place::PickPlaceExecutor>();

	RCLCPP_INFO(node->get_logger(), "Starting pick_place_executor_node...");

	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
