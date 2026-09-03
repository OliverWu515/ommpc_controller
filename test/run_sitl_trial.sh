#!/usr/bin/env bash
set -euo pipefail

usage()
{
  echo "usage: run_sitl_trial.sh TRAJECTORY OUTPUT_JSON [TASK_YAML] [POLY_PEAK_SPEED] [TRIAL_DURATION]"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if (( $# < 2 || $# > 5 )); then
  usage >&2
  exit 2
fi

trajectory="$1"
output_json="$2"
task_yaml="${3:-}"
poly_peak_speed="${4:-3.0}"
trial_duration="${5:-}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_dir="$(cd -- "${script_dir}/.." && pwd)"

if [[ "${trajectory}" == "text" ]]; then
  trajectory="horizontal-circle"
fi
if [[ "${trajectory}" != "horizontal-circle" && "${trajectory}" != "poly" ]]; then
  echo "trajectory must be horizontal-circle or poly" >&2
  exit 2
fi

if [[ -z "${task_yaml}" ]]; then
  case "${trajectory}" in
    horizontal-circle)
      task_yaml="${repository_dir}/ommpc_ros/config/params.yaml"
      ;;
    poly)
      task_yaml="${repository_dir}/ommpc_ros/config/polynomial_fixed_yaw.yaml"
      ;;
  esac
fi

if [[ ! -f "${task_yaml}" ]]; then
  echo "task configuration does not exist: ${task_yaml}" >&2
  exit 2
fi
for command_name in rosparam rosrun; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "${command_name} is unavailable; source ROS and the built workspace first" >&2
    exit 2
  fi
done

rosparam delete /ommpc_controller >/dev/null 2>&1 || true
rosparam load "${task_yaml}" /ommpc_controller

mkdir -p "$(dirname -- "${output_json}")"
controller_log="${output_json%.json}.controller.log"
rosrun ommpc_ros ommpc_example_node __name:=ommpc_controller >"${controller_log}" 2>&1 &
controller_pid=$!
cleanup()
{
  kill "${controller_pid}" >/dev/null 2>&1 || true
  wait "${controller_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

runner_arguments=(
  --trajectory "${trajectory}"
  --text-duration 27.0
  --poly-peak-speed "${poly_peak_speed}"
  --output "${output_json}"
)
if [[ -n "${trial_duration}" ]]; then
  runner_arguments+=(--trial-duration "${trial_duration}")
fi

rosrun ommpc_ros sitl_test_runner.py "${runner_arguments[@]}"
