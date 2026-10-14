#include "arm_course/ros_helpers.hpp"
#include "arm_module_07/policy.hpp"
#include <filesystem>
#include <iostream>
using namespace arm_course;
using namespace course_policy;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("policy_train");
  try {
    Policy model;
    const auto output =
        parameter<std::string>(n, "output", "/tmp/course_policy.json");
    const int epochs = parameter<int>(n, "epochs", 100);
    if (epochs < 1 || epochs > 10000)
      throw std::runtime_error("epochs must be 1..10000");
    if (parameter<bool>(n, "self_test", false)) {
      Eigen::MatrixXf x = Eigen::MatrixXf::Random(input_size, 256),
                      y(action_size, 256);
      for (int row = 0; row < action_size; ++row)
        y.row(row) = 0.3f * x.row(192 + row) + 0.2f * x.row(row) +
                     Eigen::RowVectorXf::Constant(256, row * 0.05f);
      model.normalize(x, y);
      float before = model.loss(x, y);
      model.train(x, y, epochs);
      float after = model.loss(x, y);
      model.training_source = "synthetic_self_test";
      model.save(output, {{"before", before}, {"after", after}});
      auto restored = Policy::load(output);
      if (after > before * 0.2 ||
          (restored.predict(x.col(0)) - model.predict(x.col(0))).norm() > 1e-5)
        throw std::runtime_error("Learning/serialization self-test failed");
      std::cout << "PASS: native C++ learning loss " << before << " -> "
                << after
                << "; checkpoint round-trip matches. Synthetic test is not a "
                   "trained robot policy.\n";
      rclcpp::shutdown();
      return 0;
    }
    const auto directory = parameter<std::string>(n, "dataset", "");
    if (directory.empty())
      throw std::runtime_error(
          "Provide dataset:= directory with recorded episode JSONL files");
    std::vector<std::filesystem::path> files;
    for (const auto &entry :
         std::filesystem::recursive_directory_iterator(directory))
      if (entry.path().extension() == ".jsonl")
        files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    if (files.size() < 2)
      throw std::runtime_error("At least two independent episodes required for "
                               "episode-level holdout");
    std::vector<Eigen::VectorXf> train_x, train_y, test_x, test_y;
    std::vector<std::string> train_ids, test_ids;
    const size_t split = std::max<size_t>(1, files.size() * 4 / 5);
    for (size_t episode = 0; episode < files.size(); ++episode) {
      std::ifstream stream(files[episode]);
      std::string line;
      while (std::getline(stream, line)) {
        Json sample = Json::parse(line);
        if (sample.at("robot") != "panda" ||
            sample.at("state_source") != "gazebo" || sample.at("schema") != 1)
          throw std::runtime_error(
              "Dataset schema/robot/state source mismatch");
        auto x = sample.at("features").get<std::vector<float>>(),
             y = sample.at("action").get<std::vector<float>>();
        if (x.size() != input_size || y.size() != action_size)
          throw std::runtime_error("Dataset dimensions differ from model");
        auto &xs = episode < split ? train_x : test_x;
        auto &ys = episode < split ? train_y : test_y;
        xs.emplace_back(Eigen::Map<Eigen::VectorXf>(x.data(), x.size()));
        ys.emplace_back(Eigen::Map<Eigen::VectorXf>(y.data(), y.size()));
      }
      (episode < split ? train_ids : test_ids)
          .push_back(files[episode].string());
    }
    if (train_x.size() < 32 || test_x.empty())
      throw std::runtime_error(
          "Need at least 32 training samples and a nonempty held-out episode");
    auto matrix = [](const std::vector<Eigen::VectorXf> &v) {
      Eigen::MatrixXf m(v[0].size(), v.size());
      for (size_t i = 0; i < v.size(); ++i)
        m.col(i) = v[i];
      return m;
    };
    auto x = matrix(train_x), y = matrix(train_y), tx = matrix(test_x),
         ty = matrix(test_y);
    model.train(x, y, epochs);
    const float train_loss = model.loss(x, y), test_loss = model.loss(tx, ty);
    model.save(output, {{"train_episodes", train_ids},
                        {"test_episodes", test_ids},
                        {"train_samples", train_x.size()},
                        {"test_samples", test_x.size()},
                        {"epochs", epochs},
                        {"normalized_train_mse", train_loss},
                        {"normalized_test_mse", test_loss},
                        {"action_horizon_seconds", 0.2}});
    std::cout << "Checkpoint: " << output
              << "\nTraining normalized MSE: " << train_loss
              << "\nHeld-out normalized MSE: " << test_loss
              << "\nOffline MSE does not establish manipulation success; "
                 "evaluate in Gazebo.\n";
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
