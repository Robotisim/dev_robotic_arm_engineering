#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/action/task.hpp>
#include <arm_course_interfaces/msg/objects.hpp>
#include <fstream>
#include <std_srvs/srv/trigger.hpp>
using namespace arm_course;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("project_runner");
  try {
    const auto project = parameter<std::string>(n, "project", "sort");
    if (project != "sort" && project != "stack" && project != "grasps")
      throw std::runtime_error("project must be sort, stack, or grasps");
    const bool execute = parameter<bool>(n, "execute", false);
    const int trials = parameter<int>(n, "trials", 1);
    if (trials < 1 || trials > 50)
      throw std::runtime_error("trials must be 1..50");
    std::mutex mutex;
    arm_course_interfaces::msg::Objects latest;
    auto sub = n->create_subscription<arm_course_interfaces::msg::Objects>(
        "/learning/objects", 10,
        [&](arm_course_interfaces::msg::Objects::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex);
          latest = *m;
        });
    using A = arm_course_interfaces::action::Task;
    auto client = rclcpp_action::create_client<A>(n, "/learning/task");
    auto reset =
        n->create_client<std_srvs::srv::Trigger>("/learning/reset_scene");
    Spinner spin(n);
    if (!client->wait_for_action_server(std::chrono::seconds(10)))
      throw std::runtime_error("Panda task server unavailable");
    auto observe = [&]() {
      arm_course_interfaces::msg::Objects obs;
      auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      do {
        {
          std::lock_guard<std::mutex> lock(mutex);
          obs = latest;
        }
        if (obs.header.frame_id == "panda_link0" && obs.objects.size() >= 1 &&
            (n->now() - rclcpp::Time(obs.header.stamp)).seconds() < 0.8)
          return obs;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } while (std::chrono::steady_clock::now() < end && rclcpp::ok());
      throw std::runtime_error("No fresh simulated objects");
    };
    std::ofstream report(
        parameter<std::string>(n, "report", "/tmp/panda_projects.csv"));
    if (!report)
      throw std::runtime_error("Cannot open report");
    report << "project,trial,colour,yaw,success,detail\n";
    int completed = 0, total = 0;
    for (int trial = 0; trial < trials; ++trial) {
      if (trial > 0 && execute) {
        if (!reset->wait_for_service(std::chrono::seconds(2)))
          throw std::runtime_error("Reset unavailable");
        auto f = reset->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        if (f.wait_for(std::chrono::seconds(12)) != std::future_status::ready ||
            !f.get()->success)
          throw std::runtime_error("Reset failed");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      int placed = 0;
      for (const auto &colour : {"red", "green", "blue"}) {
        auto obs = observe();
        std::vector<arm_course_interfaces::msg::Object> found;
        for (const auto &o : obs.objects)
          if (o.colour == colour)
            found.push_back(o);
        if (found.size() != 1)
          throw std::runtime_error("Expected one " + std::string(colour) +
                                   " cube");
        const auto &o = found[0];
        A::Goal goal;
        goal.operation = "pick";
        goal.execute = execute;
        goal.pose.header = obs.header;
        goal.pose.pose = o.pose;
        goal.object_id = o.id;
        goal.object_size = std::max({o.size.x, o.size.y, o.size.z});
        goal.destination.header = obs.header;
        goal.destination.pose.orientation.w = 1;
        goal.destination.pose.position.x =
            project == "stack" ? 0.6 : 0.35 + 0.12 * placed;
        goal.destination.pose.position.y = project == "stack" ? 0.25 : -0.28;
        goal.destination.pose.position.z =
            0.025 + (project == "stack" ? 0.04 * placed : 0.0);
        goal.grasp_yaw = project == "grasps" ? placed * M_PI / 4 : 0;
        auto f = client->async_send_goal(goal);
        if (f.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
          throw std::runtime_error("Acceptance timeout");
        auto h = f.get();
        if (!h)
          throw std::runtime_error("Task rejected");
        auto future = client->async_get_result(h);
        if (future.wait_for(std::chrono::seconds(180)) !=
            std::future_status::ready) {
          client->async_cancel_goal(h);
          throw std::runtime_error("Project task timed out");
        }
        auto result = future.get();
        ++total;
        completed += result.result->success;
        report << project << ',' << trial << ',' << colour << ','
               << goal.grasp_yaw << ',' << result.result->success << ",\""
               << result.result->detail << "\"\n";
        report.flush();
        if (!result.result->success) {
          RCLCPP_ERROR(n->get_logger(), "%s", result.result->detail.c_str());
          break;
        }
        ++placed;
      }
    }
    RCLCPP_INFO(n->get_logger(), "%s: %d/%d tasks succeeded", project.c_str(),
                completed, total);
    rclcpp::shutdown();
    return completed == 3 * trials ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
