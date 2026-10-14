#include <arm_course_interfaces/msg/objects.hpp>
#include <atomic>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class SceneManager : public rclcpp::Node {
  gz::transport::Node transport_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_;
  std::atomic<bool> idle_{true};
  rclcpp::Publisher<arm_course_interfaces::msg::Objects>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::mt19937 random_;

  void poses(const gz::msgs::Pose_V &msg) {
    arm_course_interfaces::msg::Objects out;
    out.header.frame_id = "panda_link0";
    out.header.stamp = now();
    for (const auto &p : msg.pose())
      for (const auto &colour : {"red", "green", "blue"})
        if (p.name() == std::string(colour) + "_cube") {
          arm_course_interfaces::msg::Object o;
          o.id = p.name();
          o.colour = colour;
          o.size.x = o.size.y = o.size.z = 0.04;
          o.confidence = 1;
          o.pose.position.x = p.position().x();
          o.pose.position.y = p.position().y();
          o.pose.position.z = p.position().z();
          o.pose.orientation.x = p.orientation().x();
          o.pose.orientation.y = p.orientation().y();
          o.pose.orientation.z = p.orientation().z();
          o.pose.orientation.w = p.orientation().w();
          out.objects.push_back(o);
        }
    if (!out.objects.empty())
      pub_->publish(out);
  }

public:
  SceneManager()
      : Node("scene_manager"), random_(declare_parameter<int>("seed", 17)) {
    declare_parameter<bool>("randomize", true);
    pub_ = create_publisher<arm_course_interfaces::msg::Objects>(
        "/learning/ground_truth", 10);
    stage_ = create_subscription<std_msgs::msg::String>(
        "/learning/stage", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          idle_ = m->data == "idle";
        });
    transport_.Subscribe("/world/course/dynamic_pose/info",
                         &SceneManager::poses, this);
    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
        "/world/course/set_pose", rclcpp::ServicesQoS(), group_);
    reset_ = create_service<std_srvs::srv::Trigger>(
        "/learning/reset_scene",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          if (!idle_) {
            response->message = "Cannot reset during a robot task";
            return;
          }
          if (!client_->wait_for_service(std::chrono::seconds(2))) {
            response->message = "Gazebo set_pose unavailable";
            return;
          }
          std::uniform_real_distribution<double> x(0.38, 0.60),
              jitter(-0.025, 0.025);
          int i = 0;
          for (const auto &colour : {"red", "green", "blue"}) {
            auto request = std::make_shared<
                ros_gz_interfaces::srv::SetEntityPose::Request>();
            request->entity.name = std::string(colour) + "_cube";
            request->entity.type = 2;
            const bool randomized = get_parameter("randomize").as_bool();
            request->pose.position.x =
                randomized ? x(random_) : (i == 1 ? 0.55 : 0.45);
            request->pose.position.y = randomized
                                           ? 0.15 - 0.15 * i + jitter(random_)
                                           : 0.12 - 0.12 * i;
            ++i;
            request->pose.position.z = 0.025;
            request->pose.orientation.w = 1;
            auto f = client_->async_send_request(request);
            if (f.wait_for(std::chrono::seconds(3)) !=
                    std::future_status::ready ||
                !f.get()->success) {
              response->message = "Cube reset failed";
              return;
            }
          }
          response->success = true;
          response->message = "Reset three cubes using deterministic PRNG "
                              "sequence; reset only while arm is idle";
        });
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<SceneManager>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
