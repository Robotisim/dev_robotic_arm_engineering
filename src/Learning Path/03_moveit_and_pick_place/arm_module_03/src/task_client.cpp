#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/action/task.hpp>
#include <yaml-cpp/yaml.h>
using namespace arm_course;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("task_client");
  try {
    using A = arm_course_interfaces::action::Task;
    auto c = rclcpp_action::create_client<A>(n, "/learning/task");
    Spinner spin(n);
    A::Goal g;
    g.operation = parameter<std::string>(n, "operation", "named");
    g.target = parameter<std::string>(n, "target", "Arm Up");
    g.joints = parameter<std::vector<double>>(n, "joints", {});
    g.execute = parameter<bool>(n, "execute", false);
    auto xyz = parameter<std::vector<double>>(n, "xyz", {});
    if (!xyz.empty()) {
      if (xyz.size() != 3)
        throw std::runtime_error("xyz must contain three values");
      g.pose.pose.position.x = xyz[0];
      g.pose.pose.position.y = xyz[1];
      g.pose.pose.position.z = xyz[2];
    }
    g.pose.header.frame_id = parameter<std::string>(n, "frame", "base_link");
    g.pose.pose.orientation.w = 1;
    auto quat = parameter<std::vector<double>>(n, "quaternion", {});
    if (!quat.empty()) {
      if (quat.size() != 4)
        throw std::runtime_error("quaternion is [x,y,z,w]");
      g.pose.pose.orientation.x = quat[0];
      g.pose.pose.orientation.y = quat[1];
      g.pose.pose.orientation.z = quat[2];
      g.pose.pose.orientation.w = quat[3];
    }
    g.object_id = parameter<std::string>(n, "object_id", "red_cube");
    g.object_size = parameter<double>(n, "object_size", 0.04);
    g.grasp_yaw = parameter<double>(n, "grasp_yaw", 0.0);
    auto destination =
        parameter<std::vector<double>>(n, "destination", {0.45, -0.25, 0.025});
    if (destination.size() != 3)
      throw std::runtime_error("destination must contain three values");
    g.destination.header.frame_id = g.pose.header.frame_id;
    g.destination.pose.position.x = destination[0];
    g.destination.pose.position.y = destination[1];
    g.destination.pose.position.z = destination[2];
    g.destination.pose.orientation.w = 1;
    if (!c->wait_for_action_server(std::chrono::seconds(10)))
      throw std::runtime_error("Start the module motion server first");
    rclcpp_action::Client<A>::SendGoalOptions options;
    options.feedback_callback = [n](auto, const auto f) {
      RCLCPP_INFO(n->get_logger(), "%s", f->stage.c_str());
    };
    auto f = c->async_send_goal(g, options);
    if (f.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
      throw std::runtime_error("Acceptance timed out");
    auto h = f.get();
    if (!h)
      throw std::runtime_error("Task rejected (server may be busy)");
    auto r = c->async_get_result(h);
    if (r.wait_for(std::chrono::seconds(300)) != std::future_status::ready) {
      c->async_cancel_goal(h);
      throw std::runtime_error("Task timeout; cancellation requested");
    }
    auto result = r.get();
    RCLCPP_INFO(n->get_logger(), "%s", result.result->detail.c_str());
    rclcpp::shutdown();
    return result.result->success ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
