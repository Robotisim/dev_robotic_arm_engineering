#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "$0")/../.."
source /opt/ros/jazzy/setup.bash
# Build existing working robot dependencies without changing their source.
colcon --log-base log/course_base build --base-paths roboflex_ros2 \
  --packages-up-to roboflex_moveit roboflex_control \
  --build-base build/course_base --install-base install/course_base \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPython_EXECUTABLE=/usr/bin/python3
source install/course_base/setup.bash
colcon --log-base log/learning build --base-paths 'Learning Path' \
  --build-base build/learning --install-base install/learning \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
