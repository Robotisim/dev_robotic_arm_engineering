# Shared C++ robot core

The ROS helper spinner services short callbacks on a single executor thread. Motion and inference waits run on their owning worker/main threads, so the callback executor remains available for state and service replies. The motion server retains its planning-scene query client for the server's lifetime instead of repeatedly creating and destroying it during grasps.

The spinner constructor waits until the executor has started before returning. This makes immediate exception cleanup safe: cancellation cannot arrive before `spin()` begins. `test/spinner_lifetime_checks.cpp` exercises 500 immediate start/unwind cycles with a 15-second test deadline.

`kinematics.hpp/.cpp` parses the repository URDF through KDL, extracts joint bounds, computes FK/Jacobian, solves bounded XYZ IK and derives a teaching DH table. No MoveIt dependency is needed.

`ros_helpers.hpp` supplies named/fresh joint-state reads, sampled quintic trajectories with velocity/acceleration timing, a controller action client and executor lifetime management. Hardware state is an estimate; this class does not invent feedback.

`test/core_checks.cpp` compares DH/KDL, checks Jacobians by finite differences, solves nearby IK targets and rejects invalid values, limits and unreachable targets. Run via the root `tools/checks.sh`.
