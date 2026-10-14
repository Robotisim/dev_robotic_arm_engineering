# Small Bash observation helpers shared by the runnable project READMEs.
# This file contains no robot application logic; nodes remain C++.
course_wait_topic() {
  local deadline=$((SECONDS + 90))
  local durability="${3:-volatile}" reliability=best_effort
  if [[ "$durability" == transient_local ]]; then reliability=reliable; fi
  # A newly launched publisher may not yet be discoverable by ros2 topic echo.
  while (( SECONDS < deadline )); do
    if timeout 10 ros2 topic echo "$1" --once --full-length --qos-durability "$durability" --qos-reliability "$reliability" > "$2" 2> "$2.wait.log"; then
      test -s "$2" && return 0
    fi
    sleep 1
  done
  cat "$2.wait.log" >&2
  echo "Timed out waiting for a message on $1" >&2
  return 1
}

course_expect_failure() {
  local logfile="$1" pattern="$2" result
  shift 2
  if "$@" > "$logfile" 2>&1; then
    cat "$logfile"
    echo 'ERROR: the invalid request unexpectedly succeeded' >&2
    return 1
  else
    result=$?
  fi
  cat "$logfile"
  # Timeouts/crashes do not count as a valid application rejection.
  test "$result" -lt 124 && grep -Ei -- "$pattern" "$logfile"
}

course_wait_clock() {
  course_wait_topic /clock "$ARTIFACTS/clock_before.yaml"
  sleep 1
  course_wait_topic /clock "$ARTIFACTS/clock_after.yaml"
  awk 'FNR == NR {if ($1 == "sec:") a += $2; if ($1 == "nanosec:") a += $2/1e9; next}
    {if ($1 == "sec:") b += $2; if ($1 == "nanosec:") b += $2/1e9}
    END {if (!(a > 0 && b > a)) {print "Simulation clock is not advancing" > "/dev/stderr"; exit 1}}' \
    "$ARTIFACTS/clock_before.yaml" "$ARTIFACTS/clock_after.yaml"
}

course_wait_controllers() {
  local deadline=$((SECONDS + 90)) name missing
  while (( SECONDS < deadline )); do
    if timeout 10 ros2 control list_controllers -c /controller_manager > "$ARTIFACTS/controllers.raw" 2> "$ARTIFACTS/controllers.wait.log"; then
      sed 's/\x1b\[[0-9;]*m//g' "$ARTIFACTS/controllers.raw" > "$ARTIFACTS/controllers.txt"
      missing=0
      for name in "$@"; do
        awk -v name="$name" '$1 == name && $3 == "active" {found=1} END {exit !found}' \
          "$ARTIFACTS/controllers.txt" || missing=1
      done
      if (( missing == 0 )); then return 0; fi
    fi
    sleep 1
  done
  cat "$ARTIFACTS/controllers.wait.log" >&2
  echo "Required controllers did not become active: $*" >&2
  return 1
}

course_panda_ready() {
  course_wait_clock
  course_wait_controllers panda_arm_controller panda_hand_controller
  course_wait_topic /learning/objects "$ARTIFACTS/objects.yaml"
  course_wait_topic /learning/ground_truth "$ARTIFACTS/ground_truth.yaml"
  course_wait_topic /learning/stage "$ARTIFACTS/stage.yaml" transient_local
  ros2 run arm_module_03 task_client --ros-args -p use_sim_time:=true \
    -p operation:=named -p target:=ready -p execute:=true
  # Address the parameter service directly rather than a cached node-name list.
  timeout 30 ros2 service call /scene_manager/set_parameters rcl_interfaces/srv/SetParameters \
    '{parameters: [{name: randomize, value: {type: 1, bool_value: false}}]}' \
    | tee "$ARTIFACTS/parameters.txt"
  grep -q 'successful=True' "$ARTIFACTS/parameters.txt"
  ros2 service call /learning/reset_scene std_srvs/srv/Trigger '{}' \
    | tee "$ARTIFACTS/reset.txt"
  grep -q 'success=True' "$ARTIFACTS/reset.txt"
  # Allow cubes to settle, then require a fresh camera observation.
  sleep 2
  course_wait_topic /learning/objects "$ARTIFACTS/objects.yaml"
}

course_check_transfers() {
  # project_runner CSV: exactly the expected number of physically verified rows.
  awk -F, -v expected="$2" 'NR > 1 {n++; if ($5 != 1) bad++}
    END {exit !(n == expected && bad == 0)}' "$1"
}
