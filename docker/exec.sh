#!/usr/bin/env bash
set -euo pipefail

container_name="${SIM_BOT_CONTAINER:-sim-bot}"

if [[ "$(docker inspect --format '{{.State.Running}}' "${container_name}" 2>/dev/null || true)" != "true" ]]; then
  echo "Container '${container_name}' is not running. Start it with ./run.sh." >&2
  exit 1
fi

tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then
  tty_args+=(-t)
fi

if (( $# == 0 )); then
  set -- bash
fi

docker exec "${tty_args[@]}" \
  "${container_name}" \
  /usr/local/bin/sim-bot-entrypoint "$@"
