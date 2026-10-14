#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/action/task.hpp>
#include <arm_course_interfaces/msg/objects.hpp>
#include <curl/curl.h>
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <regex>
using namespace arm_course;
using Json = nlohmann::json;
static size_t receive(char *data, size_t size, size_t count, void *user) {
  auto &s = *static_cast<std::string *>(user);
  if (s.size() + size * count > 1024 * 1024)
    return 0;
  s.append(data, size * count);
  return size * count;
}
static std::string base64(const std::vector<uchar> &bytes) {
  static const char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  unsigned int bits = 0;
  int used = -6;
  for (auto c : bytes) {
    bits = (bits << 8) | c;
    used += 8;
    while (used >= 0) {
      out.push_back(alphabet[(bits >> used) & 63]);
      used -= 6;
    }
  }
  if (used > -6)
    out.push_back(alphabet[((bits << 8) >> (used + 8)) & 63]);
  while (out.size() % 4)
    out += '=';
  return out;
}
static Json query(const std::string &url, const std::string &model,
                  const std::string &instruction, const cv::Mat &image) {
  if (url.rfind("http://127.0.0.1:", 0) != 0 &&
      url.rfind("http://localhost:", 0) != 0)
    throw std::runtime_error("Use a local inference endpoint");
  Json request = {
      {"model", model},
      {"stream", false},
      {"format", "json"},
      {"options", {{"temperature", 0}}},
      {"prompt",
       "Return ONLY JSON with keys action, colour, destination. Allowed "
       "action: pick. Colours: red, green, blue. Destinations: left, right. If "
       "missing/ambiguous use unknown. Default destination is left. Do not "
       "invent coordinates. Instruction: " +
           instruction}};
  if (!image.empty()) {
    std::vector<uchar> bytes;
    cv::imencode(".png", image, bytes);
    request["images"] = Json::array({base64(bytes)});
  }
  auto payload = request.dump();
  std::string response;
  CURL *curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("Cannot initialize HTTP client");
  auto headers = curl_slist_append(nullptr, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, receive);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  auto code = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  if (code != CURLE_OK || status != 200)
    throw std::runtime_error("Inference request failed: HTTP " +
                             std::to_string(status) + " / " +
                             curl_easy_strerror(code));
  auto outer = Json::parse(response);
  if (!outer.contains("response") || !outer["response"].is_string())
    throw std::runtime_error("Malformed model response");
  return Json::parse(outer["response"].get<std::string>());
}
static Json rules(std::string instruction) {
  std::transform(instruction.begin(), instruction.end(), instruction.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (!std::regex_search(instruction, std::regex("\\b(pick|grab|move)\\b")) ||
      std::regex_search(instruction, std::regex("\\b(not|don't|never)\\b")))
    throw std::runtime_error("Use an affirmative pick/move instruction");
  std::vector<std::string> colours;
  for (auto c : {"red", "green", "blue"})
    if (std::regex_search(instruction,
                          std::regex(std::string("\\b") + c + "\\b")))
      colours.push_back(c);
  if (colours.size() != 1)
    throw std::runtime_error(
        "Instruction must identify exactly one supported colour");
  bool left = instruction.find("left") != std::string::npos,
       right = instruction.find("right") != std::string::npos;
  if (left && right)
    throw std::runtime_error("Ambiguous destination");
  return {{"action", "pick"},
          {"colour", colours[0]},
          {"destination", right ? "right" : "left"}};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("language_task");
  try {
    const auto instruction = parameter<std::string>(
                   n, "instruction",
                   "pick the red block and put it in the left tray"),
               mode = parameter<std::string>(n, "interpreter", "rules");
    const bool execute = parameter<bool>(n, "execute", false),
               interpret_only = parameter<bool>(n, "interpret_only", false);
    std::mutex mutex;
    arm_course_interfaces::msg::Objects latest;
    cv::Mat image;
    auto sub = n->create_subscription<arm_course_interfaces::msg::Objects>(
        "/learning/objects", 10,
        [&](arm_course_interfaces::msg::Objects::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex);
          latest = *m;
        });
    auto camera = n->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          try {
            std::lock_guard<std::mutex> l(mutex);
            image = cv_bridge::toCvCopy(m, "bgr8")->image;
          } catch (const std::exception &e) {
            RCLCPP_WARN(n->get_logger(), "%s", e.what());
          }
        });
    Spinner spin(n);
    Json parsed;
    if (mode == "rules")
      parsed = rules(instruction);
    else if (mode == "llm" || mode == "vlm") {
      cv::Mat input;
      if (mode == "vlm") {
        auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < end) {
          {
            std::lock_guard<std::mutex> l(mutex);
            if (!image.empty()) {
              input = image.clone();
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (input.empty())
          throw std::runtime_error(
              "VLM requires a current simulated camera image");
      }
      parsed = query(parameter<std::string>(
                         n, "endpoint", "http://127.0.0.1:11434/api/generate"),
                     parameter<std::string>(n, "model", "qwen2.5:0.5b"),
                     instruction, input);
    } else
      throw std::runtime_error("interpreter must be rules, llm, or vlm");
    if (!parsed.is_object() || parsed.size() != 3 ||
        parsed.value("action", "") != "pick")
      throw std::runtime_error("Model schema/action rejected");
    const auto colour = parsed.at("colour").get<std::string>(),
               destination = parsed.at("destination").get<std::string>();
    if ((colour != "red" && colour != "green" && colour != "blue") ||
        (destination != "left" && destination != "right"))
      throw std::runtime_error("Model colour/destination rejected");
    RCLCPP_INFO(n->get_logger(), "Validated interpretation: %s",
                parsed.dump().c_str());
    if (interpret_only) {
      rclcpp::shutdown();
      return 0;
    }
    arm_course_interfaces::msg::Objects obs;
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < end) {
      {
        std::lock_guard<std::mutex> l(mutex);
        obs = latest;
      }
      if (!obs.objects.empty() &&
          (n->now() - rclcpp::Time(obs.header.stamp)).seconds() < 0.8)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (obs.header.frame_id != "panda_link0" ||
        (n->now() - rclcpp::Time(obs.header.stamp)).seconds() > 0.8)
      throw std::runtime_error("Fresh Panda simulation observations required");
    std::vector<arm_course_interfaces::msg::Object> matches;
    for (const auto &o : obs.objects)
      if (o.colour == colour)
        matches.push_back(o);
    if (matches.size() != 1)
      throw std::runtime_error("Object missing or ambiguous; no motion");
    using A = arm_course_interfaces::action::Task;
    auto client = rclcpp_action::create_client<A>(n, "/learning/task");
    if (!client->wait_for_action_server(std::chrono::seconds(5)))
      throw std::runtime_error("Task server unavailable");
    A::Goal goal;
    goal.operation = "pick";
    goal.execute = execute;
    goal.pose.header = obs.header;
    goal.pose.pose = matches[0].pose;
    goal.object_id = matches[0].id;
    goal.object_size =
        std::max({matches[0].size.x, matches[0].size.y, matches[0].size.z});
    goal.destination.header = obs.header;
    goal.destination.pose.position.x = 0.45;
    goal.destination.pose.position.y = destination == "left" ? 0.28 : -0.28;
    goal.destination.pose.position.z = 0.025;
    goal.destination.pose.orientation.w = 1;
    auto f = client->async_send_goal(goal);
    if (f.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      throw std::runtime_error("Task acceptance timeout");
    auto handle = f.get();
    if (!handle)
      throw std::runtime_error("Task rejected");
    auto result = client->async_get_result(handle);
    if (result.wait_for(std::chrono::seconds(180)) !=
        std::future_status::ready) {
      client->async_cancel_goal(handle);
      throw std::runtime_error("Task timeout");
    }
    auto r = result.get();
    RCLCPP_INFO(n->get_logger(), "%s", r.result->detail.c_str());
    rclcpp::shutdown();
    return r.result->success ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
