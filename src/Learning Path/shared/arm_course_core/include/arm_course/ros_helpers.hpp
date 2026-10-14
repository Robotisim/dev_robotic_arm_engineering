#pragma once
#include "arm_course/kinematics.hpp"
#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <fstream>
#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_course {
inline std::string read_file(const std::string &path) {
  std::ifstream f(path);
  if (!f)
    throw std::runtime_error("Cannot open " + path);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
template <class T>
T parameter(const rclcpp::Node::SharedPtr &node, const std::string &name,
            const T &fallback) {
  if (!node->has_parameter(name))
    return node->declare_parameter<T>(name, fallback);
  return node->get_parameter(name).get_value<T>();
}
class JointReader {
  std::mutex mutex_;
  sensor_msgs::msg::JointState state_;
  std::chrono::steady_clock::time_point updated_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

public:
  explicit JointReader(const rclcpp::Node::SharedPtr &n) {
    sub_ = n->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = *m;
          updated_ = std::chrono::steady_clock::now();
        });
  }
  Eigen::VectorXd get(const std::vector<std::string> &names,
                      double timeout = 5) {
    auto end = std::chrono::steady_clock::now() +
               std::chrono::duration<double>(timeout);
    do {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (std::chrono::steady_clock::now() - updated_ <
            std::chrono::seconds(1)) {
          Eigen::VectorXd q(names.size());
          bool complete = true;
          for (size_t i = 0; i < names.size(); ++i) {
            auto p =
                std::find(state_.name.begin(), state_.name.end(), names[i]);
            size_t j = std::distance(state_.name.begin(), p);
            if (p == state_.name.end() || j >= state_.position.size()) {
              complete = false;
              break;
            }
            q[i] = state_.position[j];
          }
          if (complete && q.allFinite())
            return q;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (rclcpp::ok() && std::chrono::steady_clock::now() < end);
    throw std::runtime_error("No fresh complete /joint_states");
  }
};
inline trajectory_msgs::msg::JointTrajectory
trajectory(const Kinematics &k, const Eigen::VectorXd &from,
           const Eigen::VectorXd &to, double speed = 0.25) {
  if (!k.valid(from) || !k.valid(to) || !std::isfinite(speed) || speed <= 0 ||
      speed > 1)
    throw std::runtime_error("Invalid trajectory bounds or speed");
  double duration = 1;
  for (int i = 0; i < from.size(); ++i) {
    double d = std::abs(to[i] - from[i]);
    duration =
        std::max(duration, 1.9 * d / (std::min(0.5, k.velocity()[i]) * speed));
    duration = std::max(duration, std::sqrt(6 * d / (0.4 * speed)));
  }
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = k.names();
  const int steps = std::max(2, static_cast<int>(std::ceil(duration / 0.02)));
  for (int n = 0; n <= steps; ++n) {
    double u = double(n) / steps,
           blend = 10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    for (int j = 0; j < from.size(); ++j)
      p.positions.push_back(from[j] + blend * (to[j] - from[j]));
    p.time_from_start = rclcpp::Duration::from_seconds(u * duration);
    msg.points.push_back(p);
  }
  return msg;
}
inline void execute_trajectory(
    const rclcpp::Node::SharedPtr &node, const std::string &action,
    const trajectory_msgs::msg::JointTrajectory &trajectory,
    const std::function<bool()> &cancel = []() { return false; }) {
  using A = control_msgs::action::FollowJointTrajectory;
  auto client = rclcpp_action::create_client<A>(node, action);
  if (!client->wait_for_action_server(std::chrono::seconds(5)))
    throw std::runtime_error("Controller action unavailable: " + action);
  A::Goal goal;
  goal.trajectory = trajectory;
  auto future = client->async_send_goal(goal);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    throw std::runtime_error("Controller goal acceptance timeout");
  auto handle = future.get();
  if (!handle)
    throw std::runtime_error("Controller rejected trajectory");
  auto result = client->async_get_result(handle);
  double seconds =
      rclcpp::Duration(trajectory.points.back().time_from_start).seconds() + 10;
  auto end =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (result.wait_for(std::chrono::milliseconds(50)) !=
         std::future_status::ready) {
    if (cancel() || !rclcpp::ok() || std::chrono::steady_clock::now() > end) {
      client->async_cancel_goal(handle);
      throw std::runtime_error("Trajectory cancelled or timed out");
    }
  }
  auto response = result.get();
  if (response.code != rclcpp_action::ResultCode::SUCCEEDED ||
      response.result->error_code != A::Result::SUCCESSFUL)
    throw std::runtime_error("Controller failed: " +
                             response.result->error_string);
}
class Spinner {
  // Callbacks in these lessons are short and share the default mutually
  // exclusive group. Long motion/inference work runs outside the executor.
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;

public:
  explicit Spinner(const rclcpp::Node::SharedPtr &node) {
    executor_.add_node(node);
    thread_ = std::thread([this]() { executor_.spin(); });
    // An immediate exception in the caller must not cancel before spin() has
    // set its spinning flag, or the subsequent join could wait indefinitely.
    while (rclcpp::ok() && !executor_.is_spinning())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ~Spinner() {
    executor_.cancel();
    if (thread_.joinable())
      thread_.join();
  }
};
} // namespace arm_course
