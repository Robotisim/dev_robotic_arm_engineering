#include "arm_course/ros_helpers.hpp"
#include "arm_module_07/policy.hpp"
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace course_policy;
class Recorder : public rclcpp::Node {
  struct Sample {
    double time;
    Eigen::VectorXf x;
    std::string image;
  };
  std::vector<double> state_, reference_, destination_;
  cv::Mat image_;
  double image_time_{-1}, state_time_{-1}, arm_time_{-1}, hand_time_{-1},
      last_image_{-1};
  std::string colour_, directory_, stage_{"idle"};
  std::ofstream file_;
  std::deque<Sample> pending_;
  size_t written_{0};
  bool finished_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<
      control_msgs::msg::JointTrajectoryControllerState>::SharedPtr arm_sub_,
      hand_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_;
  void tick() {
    if (finished_ || image_.empty() || state_.size() != 8 ||
        reference_.size() != 8)
      return;
    double t = now().seconds();
    if (t <= 0 || image_time_ > t + 0.05 || state_time_ > t + 0.05 ||
        arm_time_ > t + 0.05 || hand_time_ > t + 0.05) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for a simulation clock aligned with sensor/controller timestamps");
      return;
    }
    if (t - image_time_ > 0.3 || t - state_time_ > 0.3 || t - arm_time_ > 0.3 ||
        t - hand_time_ > 0.3)
      return;
    while (!pending_.empty() && t - pending_.front().time >= 0.2) {
      auto sample = pending_.front();
      pending_.pop_front();
      if (t - sample.time > 0.5)
        continue;
      Json j = {
          {"schema", 1},
          {"robot", "panda"},
          {"state_source", "gazebo"},
          {"time", sample.time},
          {"action_time", t},
          {"colour", colour_},
          {"destination", destination_},
          {"image", sample.image},
          {"features", std::vector<float>(sample.x.data(),
                                          sample.x.data() + sample.x.size())},
          {"action", reference_},
          {"stage", stage_}};
      file_ << j.dump() << '\n';
      ++written_;
    }
    if (stage_ == "idle" || image_time_ == last_image_)
      return;
    std::string filename =
        "image_" + std::to_string(static_cast<long long>(image_time_ * 1e6)) +
        ".png";
    if (!cv::imwrite(directory_ + "/" + filename, image_))
      throw std::runtime_error("Image write failed");
    pending_.push_back({image_time_,
                        features(image_, state_, colour_, destination_),
                        filename});
    last_image_ = image_time_;
    file_.flush();
  }

public:
  Recorder() : Node("episode_recorder"), reference_(8, 0) {
    if (!get_parameter("use_sim_time").as_bool())
      throw std::runtime_error(
          "Recorder is simulation-only: set use_sim_time:=true");
    colour_ = declare_parameter<std::string>("colour", "red");
    destination_ = declare_parameter<std::vector<double>>("destination",
                                                          {0.45, 0.28, 0.025});
    directory_ = declare_parameter<std::string>(
        "episode", "/tmp/arm_course_dataset/episode_001");
    if (std::filesystem::exists(directory_))
      throw std::runtime_error("Episode directory already exists; choose a new "
                               "episode to avoid overwriting data");
    std::filesystem::create_directories(directory_);
    file_.open(directory_ + "/episode.jsonl");
    if (!file_)
      throw std::runtime_error("Cannot open episode output");
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          try {
            image_ = cv_bridge::toCvCopy(m, "bgr8")->image;
            image_time_ = rclcpp::Time(m->header.stamp).seconds();
          } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "%s", e.what());
          }
        });
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          std::vector<double> q;
          for (int i = 0; i < 8; ++i) {
            std::string name = i < 7 ? "panda_joint" + std::to_string(i + 1)
                                     : "panda_finger_joint1";
            auto p = std::find(m->name.begin(), m->name.end(), name);
            size_t index = std::distance(m->name.begin(), p);
            if (index >= m->position.size())
              return;
            q.push_back(m->position[index]);
          }
          state_ = q;
          state_time_ = rclcpp::Time(m->header.stamp).seconds();
        });
    arm_sub_ =
        create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/panda_arm_controller/controller_state", rclcpp::SensorDataQoS(),
            [this](control_msgs::msg::JointTrajectoryControllerState::
                       ConstSharedPtr m) {
              for (int i = 0; i < 7; ++i) {
                auto p = std::find(m->joint_names.begin(), m->joint_names.end(),
                                   "panda_joint" + std::to_string(i + 1));
                size_t index = std::distance(m->joint_names.begin(), p);
                if (index >= m->reference.positions.size())
                  return;
                reference_[i] = m->reference.positions[index];
              }
              arm_time_ = rclcpp::Time(m->header.stamp).seconds();
            });
    hand_sub_ =
        create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/panda_hand_controller/controller_state", rclcpp::SensorDataQoS(),
            [this](control_msgs::msg::JointTrajectoryControllerState::
                       ConstSharedPtr m) {
              if (m->reference.positions.size() == 1) {
                reference_[7] = m->reference.positions[0];
                hand_time_ = rclcpp::Time(m->header.stamp).seconds();
              }
            });
    stage_sub_ = create_subscription<std_msgs::msg::String>(
        "/learning/stage", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m) { stage_ = m->data; });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      try {
        tick();
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        finished_ = true;
      }
    });
    finish_ = create_service<std_srvs::srv::Trigger>(
        "/learning/finish_episode",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          finished_ = true;
          pending_.clear();
          file_.flush();
          response->success = written_ > 0;
          response->message = "Saved " + std::to_string(written_) +
                              " samples to " + directory_ +
                              "; final look-ahead samples dropped";
        });
    RCLCPP_INFO(get_logger(), "Recording moving task stages to %s",
                directory_.c_str());
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Recorder>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
