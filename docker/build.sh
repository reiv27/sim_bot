#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
reactive_source="${REACTIVE_CIRCUMNAV_SOURCE:-${repo_root}/../reactive_circumnav}"
image_name="${SIM_BOT_IMAGE:-sim-bot:jazzy}"

if [[ ! -f "${reactive_source}/package.xml" ]]; then
  echo "reactive_circumnav source not found at '${reactive_source}'." >&2
  echo "Set REACTIVE_CIRCUMNAV_SOURCE to the package directory." >&2
  exit 1
fi

docker build \
  --file "${script_dir}/Dockerfile" \
  --tag "${image_name}" \
  --build-context "reactive_circumnav=${reactive_source}" \
  "$@" \
  "${repo_root}"
