#!/usr/bin/env bash
set -euo pipefail

image_name="${SIM_BOT_IMAGE:-sim-bot:jazzy}"
container_name="${SIM_BOT_CONTAINER:-sim-bot}"

if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  echo "Docker image '${image_name}' does not exist. Run ./build.sh first." >&2
  exit 1
fi

if docker container inspect "${container_name}" >/dev/null 2>&1; then
  echo "Container '${container_name}' already exists. Use ./restart.sh or ./stop.sh." >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set; Gazebo GUI cannot connect to the host display." >&2
  exit 1
fi

docker_args=(
  --detach
  --name "${container_name}"
  --network host
  --ipc host
  --env "DISPLAY=${DISPLAY}"
  --env QT_X11_NO_MITSHM=1
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
)

xauthority_path="${XAUTHORITY:-}"
if [[ -z "${xauthority_path}" ]]; then
  gdm_xauthority="/run/user/$(id -u)/gdm/Xauthority"
  if [[ -r "${gdm_xauthority}" ]]; then
    xauthority_path="${gdm_xauthority}"
  fi
fi

if [[ -n "${xauthority_path}" && -r "${xauthority_path}" ]]; then
  docker_args+=(
    --env XAUTHORITY=/tmp/host-xauthority
    --volume "${xauthority_path}:/tmp/host-xauthority:ro"
  )
else
  echo "Warning: no readable Xauthority file found; X11 may reject the container." >&2
fi

if [[ -d /dev/dri ]]; then
  docker_args+=(--device /dev/dri:/dev/dri)
fi

if [[ -d /dev/input ]]; then
  docker_args+=(--volume /dev/input:/dev/input:ro)
fi

docker_runtimes="$(docker info --format '{{json .Runtimes}}' 2>/dev/null || true)"
if command -v nvidia-smi >/dev/null 2>&1 && [[ "${docker_runtimes}" == *'"nvidia"'* ]]; then
  docker_args+=(
    --gpus all
    --env NVIDIA_DRIVER_CAPABILITIES=all
  )
fi

docker run \
  "${docker_args[@]}" \
  "${image_name}" \
  ros2 launch sim_bot sim_with_obstacles.launch.py "$@"

echo "Simulation container '${container_name}' started."
echo "Use ./exec.sh to enter it and ./stop.sh to stop it."
