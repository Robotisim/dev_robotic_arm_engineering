#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "$0")/../.."
source 'Learning Path/tools/setup.bash'
colcon --log-base log/learning_test test --base-paths 'Learning Path' \
  --build-base build/learning --install-base install/learning \
  --packages-select arm_course_core arm_module_07 --event-handlers console_direct+
colcon test-result --test-result-base build/learning --verbose
/usr/bin/python3 - <<'PY'
import ast
import pathlib
import xml.etree.ElementTree as ET
root = pathlib.Path('Learning Path')
for path in root.rglob('*.launch.py'):
    ast.parse(path.read_text(), filename=str(path))
for pattern in ('package.xml', '*.sdf'):
    for path in root.rglob(pattern): ET.parse(path)
print('Launch syntax and package/world XML checked.')
PY
