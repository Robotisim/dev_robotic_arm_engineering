#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace course_policy {
using Json = nlohmann::json;
constexpr int image_side = 8, input_size = 206, action_size = 8;
inline Eigen::VectorXf features(const cv::Mat &bgr,
                                const std::vector<double> &state,
                                const std::string &colour,
                                const std::vector<double> &destination) {
  if (bgr.empty() || bgr.type() != CV_8UC3 || state.size() != 8 ||
      destination.size() != 3)
    throw std::runtime_error(
        "Policy needs BGR image, eight joint values, and XYZ destination");
  cv::Mat resized;
  cv::resize(bgr, resized, {image_side, image_side}, 0, 0, cv::INTER_AREA);
  Eigen::VectorXf x(input_size);
  int k = 0;
  for (int row = 0; row < image_side; ++row)
    for (int col = 0; col < image_side; ++col) {
      auto p = resized.at<cv::Vec3b>(row, col);
      for (int channel = 2; channel >= 0; --channel)
        x[k++] = p[channel] / 255.f;
    }
  for (double q : state)
    x[k++] = q;
  if (colour != "red" && colour != "green" && colour != "blue")
    throw std::runtime_error("Unknown instruction colour");
  for (auto c : {"red", "green", "blue"})
    x[k++] = colour == c ? 1.f : 0.f;
  for (double d : destination)
    x[k++] = d;
  if (!x.allFinite())
    throw std::runtime_error("Non-finite policy input");
  return x;
}
inline Json matrix_json(const Eigen::MatrixXf &m) {
  return {{"rows", m.rows()},
          {"cols", m.cols()},
          {"data", std::vector<float>(m.data(), m.data() + m.size())}};
}
inline Eigen::MatrixXf matrix_read(const Json &j) {
  int rows = j.at("rows"), cols = j.at("cols");
  auto data = j.at("data").get<std::vector<float>>();
  if (rows < 1 || cols < 1 || rows > 10000 || cols > 10000 ||
      static_cast<size_t>(rows) * cols != data.size())
    throw std::runtime_error("Invalid checkpoint matrix shape");
  Eigen::MatrixXf m =
      Eigen::Map<const Eigen::MatrixXf>(data.data(), rows, cols);
  if (!m.allFinite())
    throw std::runtime_error("Non-finite checkpoint matrix");
  return m;
}
class Policy {
public:
  Eigen::MatrixXf w1, w2;
  Eigen::VectorXf b1, b2, xmean, xscale, ymean, yscale;
  std::string training_source = "demonstrations";
  explicit Policy(int hidden = 64)
      : w1(hidden, input_size), w2(action_size, hidden),
        b1(Eigen::VectorXf::Zero(hidden)),
        b2(Eigen::VectorXf::Zero(action_size)),
        xmean(Eigen::VectorXf::Zero(input_size)),
        xscale(Eigen::VectorXf::Ones(input_size)),
        ymean(Eigen::VectorXf::Zero(action_size)),
        yscale(Eigen::VectorXf::Ones(action_size)) {
    std::mt19937 generator(17);
    std::normal_distribution<float> normal(0, 1);
    for (auto *w : {&w1, &w2})
      for (int i = 0; i < w->size(); ++i)
        w->data()[i] = normal(generator) * std::sqrt(1.f / w->cols());
  }
  Eigen::VectorXf predict(const Eigen::VectorXf &x) const {
    if (x.size() != input_size || !x.allFinite())
      throw std::runtime_error("Invalid features");
    Eigen::VectorXf input = (x - xmean).cwiseQuotient(xscale);
    Eigen::VectorXf hidden = (w1 * input + b1).array().tanh();
    return (w2 * hidden + b2).cwiseProduct(yscale) + ymean;
  }
  void normalize(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y) {
    xmean = x.rowwise().mean();
    ymean = y.rowwise().mean();
    xscale = ((x.colwise() - xmean).array().square().rowwise().mean())
                 .sqrt()
                 .max(0.02f);
    yscale = ((y.colwise() - ymean).array().square().rowwise().mean())
                 .sqrt()
                 .max(0.01f);
  }
  float loss(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y) const {
    double error = 0;
    for (int c = 0; c < x.cols(); ++c)
      error +=
          (predict(x.col(c)) - y.col(c)).cwiseQuotient(yscale).squaredNorm();
    return error / (x.cols() * action_size);
  }
  void train(const Eigen::MatrixXf &raw_x, const Eigen::MatrixXf &raw_y,
             int epochs, float rate = 0.001f) {
    if (raw_x.rows() != input_size || raw_y.rows() != action_size ||
        raw_x.cols() != raw_y.cols() || raw_x.cols() < 2 ||
        !raw_x.allFinite() || !raw_y.allFinite())
      throw std::runtime_error("Invalid training data");
    normalize(raw_x, raw_y);
    Eigen::MatrixXf x = (raw_x.colwise() - xmean).array().colwise() /
                        xscale.array(),
                    y = (raw_y.colwise() - ymean).array().colwise() /
                        yscale.array();
    Eigen::MatrixXf mw1 = Eigen::MatrixXf::Zero(w1.rows(), w1.cols()),
                    vw1 = mw1,
                    mw2 = Eigen::MatrixXf::Zero(w2.rows(), w2.cols()),
                    vw2 = mw2;
    Eigen::VectorXf mb1 = Eigen::VectorXf::Zero(b1.size()), vb1 = mb1,
                    mb2 = Eigen::VectorXf::Zero(b2.size()), vb2 = mb2;
    std::vector<int> order(x.cols());
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 random(23);
    int step = 0;
    auto adam = [&](auto &p, auto &m, auto &v, const auto &g) {
      m = 0.9f * m + 0.1f * g;
      v = 0.999f * v + 0.001f * g.array().square().matrix();
      p.array() -= rate * (m.array() / (1 - std::pow(0.9f, step))) /
                   ((v.array() / (1 - std::pow(0.999f, step))).sqrt() + 1e-8f);
    };
    for (int epoch = 0; epoch < epochs; ++epoch) {
      std::shuffle(order.begin(), order.end(), random);
      for (int start = 0; start < x.cols(); start += 32) {
        int count = std::min(32, int(x.cols()) - start);
        Eigen::MatrixXf xb(input_size, count), yb(action_size, count);
        for (int i = 0; i < count; ++i) {
          xb.col(i) = x.col(order[start + i]);
          yb.col(i) = y.col(order[start + i]);
        }
        Eigen::MatrixXf hidden = ((w1 * xb).colwise() + b1).array().tanh();
        Eigen::MatrixXf error =
            2.f * ((w2 * hidden).colwise() + b2 - yb) / (count * action_size);
        Eigen::MatrixXf gw2 = error * hidden.transpose() + 0.00001f * w2,
                        dh = (w2.transpose() * error).array() *
                             (1 - hidden.array().square()),
                        gw1 = dh * xb.transpose() + 0.00001f * w1;
        Eigen::VectorXf gb1 = dh.rowwise().sum(), gb2 = error.rowwise().sum();
        ++step;
        adam(w1, mw1, vw1, gw1);
        adam(w2, mw2, vw2, gw2);
        adam(b1, mb1, vb1, gb1);
        adam(b2, mb2, vb2, gb2);
      }
    }
  }
  void save(const std::string &path, const Json &metadata) const {
    Json j = {{"format", 1},
              {"kind", "educational_rgb_language_state_mlp"},
              {"training_source", training_source},
              {"image_side", image_side},
              {"w1", matrix_json(w1)},
              {"w2", matrix_json(w2)},
              {"b1", matrix_json(b1)},
              {"b2", matrix_json(b2)},
              {"xmean", matrix_json(xmean)},
              {"xscale", matrix_json(xscale)},
              {"ymean", matrix_json(ymean)},
              {"yscale", matrix_json(yscale)},
              {"metadata", metadata}};
    std::ofstream f(path);
    if (!f)
      throw std::runtime_error("Cannot write checkpoint");
    f << j.dump(2);
  }
  static Policy load(const std::string &path) {
    std::ifstream f(path);
    if (!f)
      throw std::runtime_error("Checkpoint missing: " + path);
    Json j;
    f >> j;
    if (j.at("format") != 1 || j.at("image_side") != image_side ||
        j.at("kind") != "educational_rgb_language_state_mlp")
      throw std::runtime_error("Unsupported checkpoint format");
    Policy p;
    p.w1 = matrix_read(j.at("w1"));
    p.w2 = matrix_read(j.at("w2"));
    p.b1 = matrix_read(j.at("b1"));
    p.b2 = matrix_read(j.at("b2"));
    p.xmean = matrix_read(j.at("xmean"));
    p.xscale = matrix_read(j.at("xscale"));
    p.ymean = matrix_read(j.at("ymean"));
    p.yscale = matrix_read(j.at("yscale"));
    p.training_source = j.at("training_source");
    if (p.w1.cols() != input_size || p.w2.rows() != action_size ||
        p.w2.cols() != p.w1.rows() || p.b1.size() != p.w1.rows() ||
        p.b2.size() != action_size || p.xmean.size() != input_size ||
        p.xscale.size() != input_size || p.ymean.size() != action_size ||
        p.yscale.size() != action_size || (p.xscale.array() <= 0).any() ||
        (p.yscale.array() <= 0).any())
      throw std::runtime_error("Checkpoint dimensions/normalization invalid");
    return p;
  }
};
} // namespace course_policy
