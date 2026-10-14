#include "arm_course/ros_helpers.hpp"
#include <iostream>
#include <random>
static void require(bool value, const std::string &why) {
  if (!value)
    throw std::runtime_error(why);
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("Supply RoboFlex URDF path");
    arm_course::Kinematics k(arm_course::read_file(argv[1]), "base_link",
                             "gripper_drive_link");
    std::mt19937 random(91);
    std::uniform_real_distribution<double> unit(0.1, 0.9);
    double worst = 0;
    int solved = 0;
    for (int n = 0; n < 100; ++n) {
      Eigen::VectorXd q(k.names().size());
      for (int i = 0; i < q.size(); ++i)
        q[i] = k.lower()[i] + unit(random) * (k.upper()[i] - k.lower()[i]);
      auto f = k.fk(q), dh = k.dh_fk(q);
      worst = std::max(worst, (f.matrix() - dh.matrix()).norm());
      require((f.matrix() - dh.matrix()).norm() < 5e-6,
              "DH and KDL disagree beyond rounded-axis tolerance");
      auto j = k.jacobian(q);
      for (int i = 0; i < q.size(); ++i) {
        auto shifted = q;
        shifted[i] += 1e-6;
        Eigen::Vector3d derivative =
            (k.fk(shifted).translation() - f.translation()) / 1e-6;
        require((derivative - j.col(i).head(3)).norm() < 1e-5,
                "Jacobian finite difference mismatch");
      }
      auto seed = (q.array() + 0.03).matrix().cwiseMin(k.upper()).eval();
      auto ik = k.ik(f.translation(), seed);
      require(ik.success, "Nearby reachable IK failed");
      ++solved;
      auto tr = arm_course::trajectory(k, q, seed);
      for (const auto &point : tr.points) {
        Eigen::Map<const Eigen::VectorXd> values(point.positions.data(),
                                                 point.positions.size());
        require(k.valid(values), "Trajectory exceeded hard limits");
      }
    }
    auto seed = ((k.lower() + k.upper()) / 2).eval();
    require(!k.ik(Eigen::Vector3d(10, 10, 10), seed).success,
            "Unreachable goal accepted");
    auto invalid = seed;
    invalid[0] = k.upper()[0] + 0.1;
    require(!k.valid(invalid), "Hard limit not enforced");
    bool nan = false;
    try {
      k.ik(Eigen::Vector3d(NAN, 0, 0), seed);
    } catch (const std::invalid_argument &) {
      nan = true;
    }
    require(nan, "NaN accepted");
    std::cout << "PASS: 100 DH/KDL comparisons, numerical Jacobians, " << solved
              << " IK goals, trajectory bounds, unreachable/NaN rejection. "
                 "Worst DH error "
              << worst << '\n'
              << k.dh_report();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
