#include "arm_course/ros_helpers.hpp"
#include <iostream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "spinner_lifetime_checks",
      rclcpp::NodeOptions().start_parameter_services(false)
          .start_parameter_event_publisher(false));
  // Invalid commands can unwind immediately after starting a callback thread.
  // The CTest deadline makes a lost early cancellation a test failure.
  for (int attempt = 0; attempt < 500; ++attempt) {
    try {
      arm_course::Spinner spin(node);
      throw std::runtime_error("immediate application rejection");
    } catch (const std::runtime_error &) {
    }
  }
  rclcpp::shutdown();
  std::cout << "PASS: 500 immediate executor start/unwind cycles completed\n";
  return 0;
}
