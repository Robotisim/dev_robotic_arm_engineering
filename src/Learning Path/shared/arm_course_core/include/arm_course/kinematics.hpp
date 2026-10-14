#pragma once
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <string>
#include <vector>

namespace arm_course {
struct IKResult {
  bool success{false};
  Eigen::VectorXd joints;
  double residual{0.0};
  int iterations{0};
  std::string reason;
};
struct DHRow {
  double theta, d, a, alpha;
};
class Kinematics {
public:
  Kinematics(const std::string &xml, const std::string &base,
             const std::string &tip);
  Eigen::Isometry3d fk(const Eigen::VectorXd &q) const;
  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q) const;
  IKResult ik(const Eigen::Vector3d &target, const Eigen::VectorXd &seed,
              double tolerance = 1e-4, int iterations = 500) const;
  bool valid(const Eigen::VectorXd &q) const;
  Eigen::Isometry3d dh_fk(const Eigen::VectorXd &q) const;
  std::string dh_report() const;
  const std::vector<std::string> &names() const { return names_; }
  const Eigen::VectorXd &lower() const { return lower_; }
  const Eigen::VectorXd &upper() const { return upper_; }
  const Eigen::VectorXd &velocity() const { return velocity_; }

private:
  KDL::Chain chain_;
  std::vector<std::string> names_;
  Eigen::VectorXd lower_, upper_, velocity_;
  std::vector<DHRow> dh_;
  Eigen::Isometry3d dh_base_, dh_tool_;
};
} // namespace arm_course
