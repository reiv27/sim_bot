#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
params_file="${REACTIVE_CIRCUMNAV_PARAMS_FILE:-/opt/sim_bot_ws/install/share/reactive_circumnav/config/controller_config.yaml}"
telemetry_dir="${SIM_BOT_TELEMETRY_DIR:-${script_dir}/telemetry}"

mkdir -p "${telemetry_dir}"
telemetry_dir="$(cd -- "${telemetry_dir}" && pwd)"

run_id="$(date +%Y%m%d_%H%M%S_%N)"
telemetry_name="reactive_circumnav_${run_id}.csv"
timing_name="reactive_circumnav_timing_${run_id}.csv"
host_telemetry_log="${telemetry_dir}/${telemetry_name}"
host_timing_log="${telemetry_dir}/${timing_name}"

# Create files as the host user so container writes do not leave root-owned
# experiment results in the repository.
touch "${host_telemetry_log}" "${host_timing_log}"

telemetry_log="/opt/sim_bot_data/telemetry/${telemetry_name}"
timing_log="/opt/sim_bot_data/telemetry/${timing_name}"

echo "Controller telemetry: ${host_telemetry_log}"
echo "Timing telemetry:     ${host_timing_log}"

exec "${script_dir}/exec.sh" \
  ros2 run reactive_circumnav reactive_circumnav \
  --ros-args \
  --params-file "${params_file}" \
  -p "telemetry_log_path:=${telemetry_log}" \
  -p "timing_log_path:=${timing_log}" \
  "$@"
