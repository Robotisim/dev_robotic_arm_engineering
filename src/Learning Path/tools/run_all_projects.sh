#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "$0")/../.."
mkdir -p 'Learning Path/artifacts'
run_root="$(mktemp -d "$PWD/Learning Path/artifacts/readme-run-XXXXXX")"
printf 'project\texit\tevidence\n' > "$run_root/results.tsv"
failed=0
while IFS= read -r readme; do
  project="$(basename "$(dirname "$(dirname "$(dirname "$readme")")")")_$(basename "$(dirname "$readme")")"
  evidence="$run_root/$project"
  echo "Running $readme"
  if ARTIFACTS="$evidence" timeout --signal=INT --kill-after=45 1200 \
      bash 'Learning Path/tools/run_project.sh' "$readme"; then
    result=0
  else
    result=$?
    failed=$((failed + 1))
  fi
  printf '%s\t%s\t%s\n' "$readme" "$result" "$evidence" >> "$run_root/results.tsv"
done < <(find 'Learning Path' -path '*/projects/*/README.md' -print | sort)
cat "$run_root/results.tsv"
echo "$failed failed projects; complete evidence: $run_root"
test "$failed" -eq 0

