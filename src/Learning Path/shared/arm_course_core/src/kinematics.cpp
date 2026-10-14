#include "arm_course/kinematics.hpp"
#include <algorithm>
#include <iomanip>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <stdexcept>
#include <urdf/model.h>

namespace arm_course {
static Eigen::Isometry3d transform(const urdf::Pose &p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  t.linear() =
      Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z)
          .normalized()
          .toRotationMatrix();
  return t;
}
static Eigen::Isometry3d frame(const Eigen::Vector3d &o,
                               const Eigen::Vector3d &x,
                               const Eigen::Vector3d &z) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = o;
  t.linear().col(0) = x;
  t.linear().col(2) = z;
  t.linear().col(1) = z.cross(x).normalized();
  return t;
}
Kinematics::Kinematics(const std::string &xml, const std::string &base,
                       const std::string &tip) {
  urdf::Model model;
  KDL::Tree tree;
  if (!model.initString(xml) || !kdl_parser::treeFromString(xml, tree) ||
      !tree.getChain(base, tip, chain_))
    throw std::runtime_error("Cannot parse URDF chain " + base + " -> " + tip);
  std::vector<urdf::JointConstSharedPtr> joints;
  auto link = model.getLink(tip);
  while (link && link->name != base) {
    if (!link->parent_joint)
      throw std::runtime_error("Base is not an ancestor of tip");
    joints.push_back(link->parent_joint);
    link = model.getLink(link->parent_joint->parent_link_name);
  }
  std::reverse(joints.begin(), joints.end());
  std::vector<Eigen::Vector3d> origins, axes;
  std::vector<double> lo, hi, vel;
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (const auto &j : joints) {
    t = t * transform(j->parent_to_joint_origin_transform);
    if (j->type == urdf::Joint::FIXED)
      continue;
    if (j->type != urdf::Joint::REVOLUTE || !j->limits || j->mimic)
      throw std::runtime_error(
          "Arm chain requires bounded, independent revolute joints");
    names_.push_back(j->name);
    lo.push_back(j->limits->lower);
    hi.push_back(j->limits->upper);
    vel.push_back(j->limits->velocity);
    origins.push_back(t.translation());
    axes.push_back(
        (t.linear() * Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z))
            .normalized());
  }
  if (names_.empty())
    throw std::runtime_error("Empty kinematic chain");
  lower_ = Eigen::Map<Eigen::VectorXd>(lo.data(), lo.size());
  upper_ = Eigen::Map<Eigen::VectorXd>(hi.data(), hi.size());
  velocity_ = Eigen::Map<Eigen::VectorXd>(vel.data(), vel.size());
  std::vector<Eigen::Isometry3d> f;
  Eigen::Vector3d x = axes.front().unitOrthogonal().normalized();
  f.push_back(frame(origins.front(), x, axes.front()));
  for (size_t i = 1; i < axes.size(); ++i) {
    const auto &u = axes[i - 1];
    const auto &v = axes[i];
    Eigen::Vector3d delta = origins[i] - origins[i - 1];
    const double c = u.dot(v), den = 1 - c * c;
    Eigen::Vector3d a, b;
    if (den > 1e-8) {
      double s = (u.dot(delta) - c * v.dot(delta)) / den;
      double r = (c * u.dot(delta) - v.dot(delta)) / den;
      a = origins[i - 1] + s * u;
      b = origins[i] + r * v;
    } else {
      b = origins[i];
      a = origins[i - 1] + u.dot(delta) * u;
    }
    x = b - a;
    if (x.norm() < 1e-9)
      x = u.cross(v);
    if (x.norm() < 1e-9)
      x = f.back().linear().col(0);
    x = (x - v * x.dot(v)).normalized();
    f.push_back(frame(b, x, v));
  }
  f.push_back(f.back());
  dh_base_ = f.front();
  dh_tool_ = f.back().inverse() * t;
  for (size_t i = 0; i < names_.size(); ++i) {
    auto a = f[i].inverse() * f[i + 1];
    double theta = std::atan2(a(1, 0), a(0, 0));
    dh_.push_back({theta, a(2, 3),
                   std::cos(theta) * a(0, 3) + std::sin(theta) * a(1, 3),
                   std::atan2(a(2, 1), a(2, 2))});
  }
}
bool Kinematics::valid(const Eigen::VectorXd &q) const {
  return q.size() == lower_.size() && q.allFinite() &&
         (q.array() >= lower_.array() - 1e-10).all() &&
         (q.array() <= upper_.array() + 1e-10).all();
}
Eigen::Isometry3d Kinematics::fk(const Eigen::VectorXd &q) const {
  if (q.size() != lower_.size() || !q.allFinite())
    throw std::invalid_argument("Invalid FK vector");
  KDL::JntArray j(q.size());
  j.data = q;
  KDL::Frame f;
  KDL::ChainFkSolverPos_recursive solver(chain_);
  if (solver.JntToCart(j, f) < 0)
    throw std::runtime_error("FK failed");
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    t.translation()[r] = f.p[r];
    for (int c = 0; c < 3; ++c)
      t(r, c) = f.M(r, c);
  }
  return t;
}
Eigen::MatrixXd Kinematics::jacobian(const Eigen::VectorXd &q) const {
  if (q.size() != lower_.size() || !q.allFinite())
    throw std::invalid_argument("Invalid Jacobian vector");
  KDL::JntArray j(q.size());
  j.data = q;
  KDL::Jacobian jac(q.size());
  KDL::ChainJntToJacSolver solver(chain_);
  if (solver.JntToJac(j, jac) < 0)
    throw std::runtime_error("Jacobian failed");
  return jac.data;
}
IKResult Kinematics::ik(const Eigen::Vector3d &target,
                        const Eigen::VectorXd &seed, double tolerance,
                        int iterations) const {
  if (!target.allFinite() || !valid(seed) || !std::isfinite(tolerance) ||
      tolerance <= 0 || iterations <= 0)
    throw std::invalid_argument("Invalid IK target, seed, or tolerance");
  IKResult out;
  out.joints = seed;
  for (int i = 0; i < iterations; ++i) {
    Eigen::Vector3d e = target - fk(out.joints).translation();
    out.residual = e.norm();
    out.iterations = i;
    if (out.residual < tolerance) {
      out.success = true;
      out.reason = "position tolerance reached";
      return out;
    }
    Eigen::MatrixXd j = jacobian(out.joints).topRows(3);
    Eigen::VectorXd dq =
        j.transpose() * (j * j.transpose() + 1e-5 * Eigen::Matrix3d::Identity())
                            .ldlt()
                            .solve(e);
    if (dq.norm() > 0.15)
      dq *= 0.15 / dq.norm();
    bool improved = false;
    for (double scale = 1; scale >= 0.015625; scale *= 0.5) {
      Eigen::VectorXd candidate =
          (out.joints + scale * dq).cwiseMax(lower_).cwiseMin(upper_);
      if ((target - fk(candidate).translation()).norm() < out.residual) {
        out.joints = candidate;
        improved = true;
        break;
      }
    }
    if (!improved)
      break;
  }
  out.residual = (target - fk(out.joints).translation()).norm();
  out.reason = "No bounded solution from this seed; try another seed or a "
               "reachable target";
  return out;
}
Eigen::Isometry3d Kinematics::dh_fk(const Eigen::VectorXd &q) const {
  if (q.size() != static_cast<int>(dh_.size()) || !q.allFinite())
    throw std::invalid_argument("Invalid DH vector");
  Eigen::Isometry3d t = dh_base_;
  for (size_t i = 0; i < dh_.size(); ++i) {
    const auto &r = dh_[i];
    t = t * Eigen::AngleAxisd(q[i] + r.theta, Eigen::Vector3d::UnitZ()) *
        Eigen::Translation3d(0, 0, r.d) * Eigen::Translation3d(r.a, 0, 0) *
        Eigen::AngleAxisd(r.alpha, Eigen::Vector3d::UnitX());
  }
  return t * dh_tool_;
}
std::string Kinematics::dh_report() const {
  std::ostringstream s;
  s << std::setprecision(12)
    << "joint,theta_offset_rad,d_m,a_m,alpha_rad,lower_rad,upper_rad\n";
  for (size_t i = 0; i < dh_.size(); ++i) {
    const auto &r = dh_[i];
    s << names_[i] << ',' << r.theta << ',' << r.d << ',' << r.a << ','
      << r.alpha << ',' << lower_[i] << ',' << upper_[i] << '\n';
  }
  s << "base_transform\n"
    << dh_base_.matrix() << "\ntool_transform\n"
    << dh_tool_.matrix() << '\n';
  return s.str();
}
} // namespace arm_course
