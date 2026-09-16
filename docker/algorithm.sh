#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
params_file="${REACTIVE_CIRCUMNAV_PARAMS_FILE:-/opt/sim_bot_ws/install/share/reactive_circumnav/config/controller_config.yaml}"
telemetry_log="${REACTIVE_CIRCUMNAV_TELEMETRY_LOG:-/tmp/reactive_circumnav.csv}"

exec "${script_dir}/exec.sh" \
  ros2 run reactive_circumnav reactive_circumnav \
  --ros-args \
  --params-file "${params_file}" \
  -p "telemetry_log_path:=${telemetry_log}" \
  "$@"
