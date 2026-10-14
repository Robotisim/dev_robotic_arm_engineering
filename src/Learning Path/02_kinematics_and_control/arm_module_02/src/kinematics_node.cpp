#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/action/move_xyz.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
using namespace arm_course;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>(
      "kinematics_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  try {
    const auto base = parameter<std::string>(n, "base_frame", "base_link"),
               tip = parameter<std::string>(n, "tool_frame",
                                            "gripper_drive_link");
    Kinematics k(parameter<std::string>(n, "robot_description", ""), base, tip);
    if (parameter<bool>(n, "dh_report", false)) {
      std::cout << k.dh_report();
      rclcpp::shutdown();
      return 0;
    }
    JointReader reader(n);
    auto pose = n->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/learning/fk_pose", 10);
    std::atomic<bool> busy{false};
    std::thread worker;
    using A = arm_course_interfaces::action::MoveXYZ;
    using H = rclcpp_action::ServerGoalHandle<A>;
    auto server = rclcpp_action::create_server<A>(
        n, "/learning/move_xyz",
        [&](const auto &, std::shared_ptr<const A::Goal> g) {
          if (g->target.header.frame_id != base ||
              !std::isfinite(g->target.point.x) ||
              !std::isfinite(g->target.point.y) ||
              !std::isfinite(g->target.point.z) || g->tolerance <= 0 ||
              !std::isfinite(g->tolerance) || g->tolerance > 0.01 ||
              busy.exchange(true))
            return rclcpp_action::GoalResponse::REJECT;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<H>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](std::shared_ptr<H> h) {
          if (worker.joinable())
            worker.join();
          worker = std::thread([&, h]() {
            auto result = std::make_shared<A::Result>();
            try {
              auto g = h->get_goal();
              auto q = reader.get(k.names());
              auto ik = k.ik(
                  {g->target.point.x, g->target.point.y, g->target.point.z}, q,
                  g->tolerance);
              result->residual = ik.residual;
              result->joints.assign(ik.joints.data(),
                                    ik.joints.data() + ik.joints.size());
              if (!ik.success)
                throw std::runtime_error(ik.reason);
              auto feedback = std::make_shared<A::Feedback>();
              feedback->stage = "IK solved; generating timed joint trajectory";
              h->publish_feedback(feedback);
              if (g->execute)
                execute_trajectory(
                    n,
                    parameter<std::string>(
                        n, "controller_action",
                        "/arm_controller/follow_joint_trajectory"),
                    trajectory(k, q, ik.joints),
                    [&]() { return h->is_canceling(); });
              result->success = true;
              result->detail = g->execute
                                   ? "Controller completed command sequence; "
                                     "hardware arrival is unmeasured"
                                   : "IK preview complete; no motion commanded";
              if (h->is_canceling()) {
                result->success = false;
                h->canceled(result);
              } else
                h->succeed(result);
            } catch (const std::exception &e) {
              result->detail = e.what();
              if (h->is_canceling())
                h->canceled(result);
              else
                h->abort(result);
            }
            busy = false;
          });
        });
    auto timer = n->create_wall_timer(std::chrono::milliseconds(200), [&]() {
      try {
        auto q = reader.get(k.names(), 0);
        auto t = k.fk(q);
        geometry_msgs::msg::PoseStamped m;
        m.header.frame_id = base;
        m.header.stamp = n->now();
        m.pose.position.x = t.translation().x();
        m.pose.position.y = t.translation().y();
        m.pose.position.z = t.translation().z();
        Eigen::Quaterniond r(t.linear());
        m.pose.orientation.x = r.x();
        m.pose.orientation.y = r.y();
        m.pose.orientation.z = r.z();
        m.pose.orientation.w = r.w();
        pose->publish(m);
      } catch (const std::exception &) {
      }
    });
    {
      Spinner spin(n);
      while (rclcpp::ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (worker.joinable())
      worker.join();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
