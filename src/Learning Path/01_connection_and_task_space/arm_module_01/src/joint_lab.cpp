#include "arm_course/ros_helpers.hpp"
#include <iostream>
#include <random>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
using namespace arm_course;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "joint_lab",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  try {
    auto xml = parameter<std::string>(node, "robot_description", "");
    Kinematics k(
        xml, parameter<std::string>(node, "base_frame", "base_link"),
        parameter<std::string>(node, "tool_frame", "gripper_drive_link"));
    const auto operation = parameter<std::string>(node, "operation", "display");
    const auto initial = parameter<std::vector<double>>(
        node, "initial", {0.0349066, -0.3839724, -0.9250245, -0.3665191});
    if (initial.size() != k.names().size())
      throw std::runtime_error("Initial joint vector has wrong size");
    Eigen::VectorXd q =
        Eigen::Map<const Eigen::VectorXd>(initial.data(), initial.size());
    if (!k.valid(q))
      throw std::runtime_error("Initial pose exceeds model limits");
    auto states = node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);
    auto diagnostics = node->create_publisher<std_msgs::msg::String>(
        "/learning/state_source", rclcpp::QoS(1).transient_local());
    std_msgs::msg::String source;
    source.data = operation == "display"
                      ? "synthetic display angles; no motors connected"
                      : "controller state; hardware values are command-derived";
    diagnostics->publish(source);
    if (operation == "display") {
      std::mutex mutex;
      auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
          "/learning/display_target", 10,
          [&](sensor_msgs::msg::JointState::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(mutex);
            auto next = q;
            if (m->name.size() != m->position.size())
              return;
            for (size_t i = 0; i < m->name.size(); ++i) {
              auto p =
                  std::find(k.names().begin(), k.names().end(), m->name[i]);
              if (p == k.names().end())
                return;
              next[std::distance(k.names().begin(), p)] = m->position[i];
            }
            if (k.valid(next))
              q = next;
            else
              RCLCPP_WARN(node->get_logger(),
                          "Rejected target outside hard joint limits");
          });
      auto timer =
          node->create_wall_timer(std::chrono::milliseconds(40), [&]() {
            std::lock_guard<std::mutex> lock(mutex);
            sensor_msgs::msg::JointState m;
            m.header.stamp = node->now();
            m.name = k.names();
            m.position.assign(q.data(), q.data() + q.size());
            m.name.push_back("joint_gripper");
            m.position.push_back(0);
            states->publish(m);
          });
      rclcpp::spin(node);
    } else if (operation == "workspace") {
      auto pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/learning/workspace", rclcpp::QoS(1).transient_local());
      visualization_msgs::msg::Marker m;
      m.header.frame_id =
          parameter<std::string>(node, "base_frame", "base_link");
      m.ns = "reachable_samples";
      m.id = 0;
      m.type = m.POINTS;
      m.action = m.ADD;
      m.pose.orientation.w = 1;
      m.scale.x = m.scale.y = 0.003;
      m.color.g = 0.8;
      m.color.a = 0.6;
      std::mt19937 random(17);
      std::uniform_real_distribution<double> unit(0, 1);
      for (int s = 0; s < 2000; ++s) {
        for (int i = 0; i < q.size(); ++i)
          q[i] = k.lower()[i] + unit(random) * (k.upper()[i] - k.lower()[i]);
        Eigen::Vector3d p = k.fk(q).translation();
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        m.points.push_back(point);
      }
      visualization_msgs::msg::MarkerArray array;
      array.markers.push_back(m);
      pub->publish(array);
      rclcpp::spin(node);
    } else {
      JointReader reader(node);
      Spinner spin(node);
      q = reader.get(k.names());
      if (operation == "jog") {
        auto name = parameter<std::string>(node, "joint", "joint_1");
        auto p = std::find(k.names().begin(), k.names().end(), name);
        if (p == k.names().end())
          throw std::runtime_error("Unknown joint");
        auto target = q;
        target[std::distance(k.names().begin(), p)] = parameter<double>(
            node, "angle", q[std::distance(k.names().begin(), p)]);
        auto tr =
            trajectory(k, q, target, parameter<double>(node, "speed", 0.25));
        if (parameter<bool>(node, "execute", false))
          execute_trajectory(node, "/arm_controller/follow_joint_trajectory",
                             tr);
        else
          RCLCPP_INFO(
              node->get_logger(),
              "Validated jog preview; execute:=true is required to dispatch");
      } else if (operation != "inspect")
        throw std::runtime_error(
            "operation must be display, workspace, inspect, or jog");
      for (int i = 0; i < q.size(); ++i)
        std::cout << k.names()[i] << ": " << q[i] << " rad ("
                  << q[i] * 180 / M_PI << " deg), bounds [" << k.lower()[i]
                  << ", " << k.upper()[i] << "]\n";
      std::cout << "TCP in base: " << k.fk(q).translation().transpose() << '\n';
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
