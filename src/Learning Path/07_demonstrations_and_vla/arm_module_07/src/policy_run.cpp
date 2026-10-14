#include "arm_course/ros_helpers.hpp"
#include "arm_module_07/policy.hpp"
#include <arm_course_interfaces/msg/objects.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <std_srvs/srv/set_bool.hpp>
using namespace arm_course;
using namespace course_policy;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>(
      "policy_run",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  bool claimed = false;
  auto owner =
      n->create_client<std_srvs::srv::SetBool>("/learning/policy_control");
  auto release = [&]() {
    if (claimed && rclcpp::ok()) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      owner->async_send_request(request).wait_for(std::chrono::seconds(2));
      claimed = false;
    }
  };
  try {
    if (!n->get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("Policy execution is simulation-only");
    Policy policy = Policy::load(
        parameter<std::string>(n, "checkpoint", "/tmp/course_policy.json"));
    if (policy.training_source != "demonstrations")
      throw std::runtime_error(
          "Refusing synthetic self-test checkpoint for robot control");
    Kinematics k(parameter<std::string>(n, "robot_description", ""),
                 "panda_link0", "panda_hand_tcp");
    if (k.names().size() != 7)
      throw std::runtime_error("Panda seven-joint model required");
    auto colour = parameter<std::string>(n, "colour", "red");
    auto destination =
        parameter<std::vector<double>>(n, "destination", {0.45, 0.28, 0.025});
    bool execute = parameter<bool>(n, "execute", false);
    int steps = parameter<int>(n, "steps", 50);
    if (steps < 1 || steps > 1000)
      throw std::runtime_error("steps must be 1..1000");
    std::mutex mutex;
    cv::Mat image;
    double image_time = -1, truth_time = -1;
    geometry_msgs::msg::Point object_position;
    double initial_z = -1, maximum_z = -1;
    auto camera = n->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          try {
            std::lock_guard<std::mutex> l(mutex);
            image = cv_bridge::toCvCopy(m, "bgr8")->image;
            image_time = rclcpp::Time(m->header.stamp).seconds();
          } catch (const std::exception &e) {
            RCLCPP_WARN(n->get_logger(), "%s", e.what());
          }
        });
    auto truth = n->create_subscription<arm_course_interfaces::msg::Objects>(
        "/learning/ground_truth", 10,
        [&](arm_course_interfaces::msg::Objects::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex);
          truth_time = rclcpp::Time(m->header.stamp).seconds();
          for (const auto &o : m->objects)
            if (o.colour == colour) {
              object_position = o.pose.position;
              if (initial_z < 0)
                initial_z = object_position.z;
              maximum_z = std::max(maximum_z, object_position.z);
            }
        });
    JointReader reader(n);
    Spinner spin(n);
    auto validity = n->create_client<moveit_msgs::srv::GetStateValidity>(
        "/check_state_validity");
    auto names = k.names();
    names.push_back("panda_finger_joint1");
    auto q = reader.get(names);
    auto ready_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (rclcpp::ok()) {
      bool ready;
      {
        std::lock_guard<std::mutex> l(mutex);
        ready = !image.empty() && truth_time >= 0;
      }
      if (ready)
        break;
      if (std::chrono::steady_clock::now() > ready_deadline)
        throw std::runtime_error(
            "Timed out waiting for simulated camera/evaluator");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    double total_latency = 0;
    int completed_steps = 0;
    if (execute) {
      if (!owner->wait_for_service(std::chrono::seconds(5)))
        throw std::runtime_error("Task server ownership service unavailable");
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      auto f = owner->async_send_request(request);
      if (f.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
          !f.get()->success)
        throw std::runtime_error("Another command owner is active");
      claimed = true;
      if (!validity->wait_for_service(std::chrono::seconds(5)))
        throw std::runtime_error("Collision checking service unavailable");
    }
    for (int step = 0; step < steps && rclcpp::ok(); ++step) {
      cv::Mat input;
      {
        std::lock_guard<std::mutex> l(mutex);
        if (n->now().seconds() - image_time > 0.5 ||
            n->now().seconds() - truth_time > 1 || image.empty())
          throw std::runtime_error(
              "Fresh simulated RGB and evaluator heartbeat required");
        input = image.clone();
      }
      q = reader.get(names);
      std::vector<double> state(q.data(), q.data() + q.size());
      auto start = std::chrono::steady_clock::now();
      auto predicted =
          policy.predict(features(input, state, colour, destination));
      double latency = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
      Eigen::VectorXd target = predicted.cast<double>();
      if (!k.valid(target.head(7)) || target[7] < 0 || target[7] > 0.04)
        throw std::runtime_error(
            "Predicted action exceeds joint/gripper bounds");
      for (int i = 0; i < 8; ++i) {
        const double max_step = i < 7 ? 0.015 : 0.003;
        target[i] = q[i] + std::clamp(target[i] - q[i], -max_step, max_step);
      }
      if (execute) {
        for (int sample = 1; sample <= 3; ++sample) {
          auto request =
              std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
          request->robot_state.is_diff = true;
          request->robot_state.joint_state.name = names;
          Eigen::VectorXd v = q + (target - q) * (sample / 3.0);
          request->robot_state.joint_state.position.assign(v.data(),
                                                           v.data() + v.size());
          auto future = validity->async_send_request(request);
          if (future.wait_for(std::chrono::seconds(2)) !=
                  std::future_status::ready ||
              !future.get()->valid)
            throw std::runtime_error(
                "Policy motion segment failed collision check");
        }
        trajectory_msgs::msg::JointTrajectory arm, hand;
        arm.joint_names = k.names();
        hand.joint_names = {"panda_finger_joint1"};
        trajectory_msgs::msg::JointTrajectoryPoint a0, a1, h0, h1;
        a0.positions.assign(q.data(), q.data() + 7);
        a1.positions.assign(target.data(), target.data() + 7);
        a1.time_from_start = rclcpp::Duration::from_seconds(0.2);
        h0.positions = {q[7]};
        h1.positions = {target[7]};
        h1.time_from_start = a1.time_from_start;
        arm.points = {a0, a1};
        hand.points = {h0, h1};
        auto gripper = std::async(std::launch::async, [&]() {
          execute_trajectory(
              n, "/panda_hand_controller/follow_joint_trajectory", hand);
        });
        execute_trajectory(n, "/panda_arm_controller/follow_joint_trajectory",
                           arm);
        gripper.get();
      }
      RCLCPP_INFO(n->get_logger(),
                  "step=%d native inference=%.3fms dispatched=%s", step,
                  latency * 1000, execute ? "yes" : "preview");
      total_latency += latency;
      ++completed_steps;
      if (!execute)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    {
      std::lock_guard<std::mutex> l(mutex);
      const double xy = std::hypot(object_position.x - destination[0],
                                   object_position.y - destination[1]);
      const double z_error = std::abs(object_position.z - destination[2]);
      const bool success = execute && maximum_z - initial_z >= 0.04 &&
                           xy <= 0.05 && z_error <= 0.025;
      std::ofstream report(
          parameter<std::string>(n, "report", "/tmp/policy_rollout.csv"));
      if (!report)
        throw std::runtime_error("Cannot open rollout report");
      report << "execute,steps,mean_inference_ms,lift_m,xy_error_m,z_error_m,"
                "manipulation_success\n"
             << execute << ',' << completed_steps << ','
             << 1000 * total_latency / std::max(1, completed_steps) << ','
             << maximum_z - initial_z << ',' << xy << ',' << z_error << ','
             << success << '\n';
      RCLCPP_INFO(n->get_logger(),
                  "Rollout complete: physical manipulation success=%s "
                  "(separate from inference completion)",
                  success ? "yes" : "no");
    }
    release();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    Spinner cleanup(n);
    release();
    rclcpp::shutdown();
    return 1;
  }
}
