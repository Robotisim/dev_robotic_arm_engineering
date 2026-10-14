# Source this file from any directory. Defaults isolate the course from hardware.
COURSE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$COURSE_ROOT/install/setup.bash" ]]; then source "$COURSE_ROOT/install/setup.bash"; fi
if [[ -f "$COURSE_ROOT/install/course_base/setup.bash" ]]; then source "$COURSE_ROOT/install/course_base/setup.bash"; fi
if [[ -f "$COURSE_ROOT/install/learning/setup.bash" ]]; then source "$COURSE_ROOT/install/learning/setup.bash"; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-81}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export GZ_PARTITION="${GZ_PARTITION:-roboflex_course}"
