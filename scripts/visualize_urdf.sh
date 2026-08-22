#!/usr/bin/env bash
# Start the default static mfr3duo URDF in RViz with GUI joint controls.
set -euo pipefail

readonly package_name="mfr3duo_description"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS 2 is not sourced. Run: source /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

if ! package_prefix="$(ros2 pkg prefix "${package_name}" 2>/dev/null)"; then
  echo "${package_name} is not discoverable. Build the workspace and source install/setup.bash." >&2
  exit 1
fi

readonly urdf_path="${package_prefix}/share/${package_name}/urdf/mfr3duo.urdf"
readonly rviz_config="${package_prefix}/share/${package_name}/rviz/mfr3duo.rviz"

for required_path in "${urdf_path}" "${rviz_config}"; do
  if [[ ! -f "${required_path}" ]]; then
    echo "Required package resource is missing: ${required_path}" >&2
    exit 1
  fi
done

readonly params_file="$(mktemp /tmp/mfr3duo_robot_description.XXXXXX.yaml)"
declare -a child_pids=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${child_pids[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  for pid in "${child_pids[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
  rm -f "${params_file}"
  exit "${status}"
}
trap cleanup EXIT INT TERM

# Write the XML as a YAML block scalar. Do not pass it through `ros2 run -p`:
# the ROS 2 CLI parameter parser is not reliable for a multi-line URDF.
python3 - "${urdf_path}" "${params_file}" <<'PY'
from pathlib import Path
import sys

urdf = Path(sys.argv[1]).read_text()
params = (
    "/**:\n"
    "  ros__parameters:\n"
    "    robot_description: |-\n"
    + "\n".join(f"      {line}" for line in urdf.splitlines())
    + "\n"
)
Path(sys.argv[2]).write_text(params)
PY

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file "${params_file}" &
child_pids+=("$!")

ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args --params-file "${params_file}" &
child_pids+=("$!")

rviz2 -d "${rviz_config}"
