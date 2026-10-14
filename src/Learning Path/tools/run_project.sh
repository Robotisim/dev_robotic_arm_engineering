#!/usr/bin/env bash
# Execute the setup, launch and run Bash blocks verbatim from one project README.
set -eo pipefail
cd -- "$(dirname -- "$0")/../.."
if [[ $# != 1 || ! -f "$1" ]]; then
  echo "Usage: bash 'Learning Path/tools/run_project.sh' 'Learning Path/MODULE/projects/PROJECT/README.md'" >&2
  exit 2
fi
readme="$(realpath "$1")"
extract() {
  awk -v marker="<!-- runnable: $1 -->" '
    $0 == marker {found=1; next}
    found && $0 == "```bash" {inside=1; next}
    inside && $0 == "```" {exit}
    inside {print}
  ' "$readme"
}
setup="$(extract setup)"
launch="$(extract launch)"
run="$(extract run)"
if [[ -z "$setup" || -z "$run" ]]; then
  echo "Missing runnable setup/run block in $readme" >&2
  exit 2
fi
# Source the README setup in this shell so its artifact path is used for all logs.
eval "$setup"
export ARTIFACTS
backend_pid=''
cleanup() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$backend_pid" ]]; then
    kill -INT -- "-$backend_pid" 2>/dev/null || true
    for ((i=0; i<20; i++)); do
      kill -0 -- "-$backend_pid" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-$backend_pid" 2>/dev/null || true
    for ((i=0; i<5; i++)); do
      kill -0 -- "-$backend_pid" 2>/dev/null || break
      sleep 1
    done
    # A simulator can outlive its launch parent. Reap only our own session.
    kill -KILL -- "-$backend_pid" 2>/dev/null || true
    wait "$backend_pid" 2>/dev/null || true
  fi
  printf '%s\t%s\t%s\n' "$result" "$readme" "$ARTIFACTS" > "$ARTIFACTS/result.tsv"
  echo "Project exit=$result; evidence: $ARTIFACTS"
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ -n "$launch" ]]; then
  # Only this newly created process group is stopped by cleanup.
  setsid env --default-signal=INT bash -c "$setup
exec $launch" > "$ARTIFACTS/backend.log" 2>&1 &
  backend_pid=$!
fi
printf '%s\n' "$setup" "$launch" "$run" > "$ARTIFACTS/commands.bash"
# pipefail propagates any failed exercise; setup enables errexit in the child.
bash -c "$setup
$run" 2>&1 | tee "$ARTIFACTS/run.log"
