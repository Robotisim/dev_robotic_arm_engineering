#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/action/task.hpp>
#include <arm_course_interfaces/msg/objects.hpp>
#include <fstream>
using namespace arm_course;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("perception_pick");
  try {
    const auto colour = parameter<std::string>(n, "colour", "red");
    const int trials = parameter<int>(n, "trials", 1),
              attempts = parameter<int>(n, "attempts", 2);
    bool execute = parameter<bool>(n, "execute", false);
    if (trials < 1 || trials > 50 || attempts < 1 || attempts > 4)
      throw std::runtime_error("trials 1..50 and attempts 1..4 required");
    const auto destination =
        parameter<std::vector<double>>(n, "destination", {0.45, -0.28, 0.025});
    if (destination.size() != 3)
      throw std::runtime_error("Destination requires xyz");
    std::mutex mutex;
    arm_course_interfaces::msg::Objects observation;
    auto sub = n->create_subscription<arm_course_interfaces::msg::Objects>(
        "/learning/objects", 10,
        [&](arm_course_interfaces::msg::Objects::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex);
          observation = *m;
        });
    using A = arm_course_interfaces::action::Task;
    auto client = rclcpp_action::create_client<A>(n, "/learning/task");
    Spinner spin(n);
    if (!client->wait_for_action_server(std::chrono::seconds(15)))
      throw std::runtime_error("Panda task server unavailable");
    std::ofstream log(
        parameter<std::string>(n, "report", "/tmp/perception_trials.csv"));
    if (!log)
      throw std::runtime_error("Cannot write report");
    log << "trial,attempt,object,success,detail\n";
    int successes = 0;
    for (int trial = 0; trial < trials && rclcpp::ok(); ++trial) {
      bool success = false;
      for (int attempt = 0; attempt < attempts && !success; ++attempt) {
        arm_course_interfaces::msg::Objects obs;
        auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
          {
            std::lock_guard<std::mutex> l(mutex);
            obs = observation;
          }
          if (!obs.objects.empty() &&
              (n->now() - rclcpp::Time(obs.header.stamp)).seconds() < 0.8)
            break;
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (obs.header.frame_id != "panda_link0" ||
            (n->now() - rclcpp::Time(obs.header.stamp)).seconds() > 0.8)
          throw std::runtime_error(
              "Fresh Panda simulation observations required");
        std::vector<arm_course_interfaces::msg::Object> matches;
        for (const auto &o : obs.objects)
          if (colour == "any" || o.colour == colour)
            matches.push_back(o);
        if (matches.empty() || (colour != "any" && matches.size() > 1))
          throw std::runtime_error("Missing or ambiguous colour target");
        const auto &o = matches.front();
        A::Goal goal;
        goal.operation = "pick";
        goal.execute = execute;
        goal.object_id = o.id;
        goal.object_size = std::max({o.size.x, o.size.y, o.size.z});
        goal.pose.header = obs.header;
        goal.pose.pose = o.pose;
        goal.destination.header = obs.header;
        goal.destination.pose.position.x = destination[0];
        goal.destination.pose.position.y = destination[1];
        goal.destination.pose.position.z = destination[2];
        goal.grasp_yaw = attempt * M_PI / 2;
        auto f = client->async_send_goal(goal);
        if (f.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
          throw std::runtime_error("Task acceptance timeout");
        auto h = f.get();
        if (!h)
          throw std::runtime_error("Task rejected");
        auto result = client->async_get_result(h);
        if (result.wait_for(std::chrono::seconds(180)) !=
            std::future_status::ready) {
          client->async_cancel_goal(h);
          throw std::runtime_error("Pick timeout");
        }
        auto r = result.get();
        success = r.result->success;
        log << trial << ',' << attempt << ',' << o.id << ',' << success << ",\""
            << r.result->detail << "\"\n";
        log.flush();
        RCLCPP_INFO(n->get_logger(), "trial %d: %s", trial,
                    r.result->detail.c_str());
      }
      successes += success;
    }
    RCLCPP_INFO(n->get_logger(), "%d/%d successful", successes, trials);
    rclcpp::shutdown();
    return successes == trials ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
