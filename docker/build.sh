#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
image_name="${SIM_BOT_IMAGE:-sim-bot:jazzy}"

docker build \
  --file "${script_dir}/Dockerfile" \
  --tag "${image_name}" \
  "$@" \
  "${repo_root}"
