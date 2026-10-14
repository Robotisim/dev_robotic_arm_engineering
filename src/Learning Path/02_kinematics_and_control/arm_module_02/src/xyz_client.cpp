#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/action/move_xyz.hpp>
using namespace arm_course;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("xyz_client");
  try {
    using A = arm_course_interfaces::action::MoveXYZ;
    auto c = rclcpp_action::create_client<A>(n, "/learning/move_xyz");
    Spinner spin(n);
    A::Goal goal;
    goal.target.header.frame_id =
        parameter<std::string>(n, "frame", "base_link");
    auto xyz = parameter<std::vector<double>>(n, "xyz", {});
    if (xyz.size() != 3)
      throw std::runtime_error("Provide xyz:=[x,y,z] in metres");
    goal.target.point.x = xyz[0];
    goal.target.point.y = xyz[1];
    goal.target.point.z = xyz[2];
    goal.tolerance = parameter<double>(n, "tolerance", 0.001);
    goal.execute = parameter<bool>(n, "execute", false);
    if (!c->wait_for_action_server(std::chrono::seconds(5)))
      throw std::runtime_error("Start Module 2 first");
    auto f = c->async_send_goal(goal);
    if (f.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      throw std::runtime_error("Goal timeout");
    auto h = f.get();
    if (!h)
      throw std::runtime_error("Goal rejected");
    auto r = c->async_get_result(h);
    if (r.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
      c->async_cancel_goal(h);
      throw std::runtime_error("Execution timed out");
    }
    auto result = r.get();
    RCLCPP_INFO(n->get_logger(), "%s residual=%.6f",
                result.result->detail.c_str(), result.result->residual);
    rclcpp::shutdown();
    return result.result->success ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
