# syntax=docker/dockerfile:1
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-yaml \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-joy \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-xacro \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/sim_bot_ws

COPY . src/sim_bot

COPY --from=reactive_circumnav CMakeLists.txt package.xml src/reactive_circumnav/
COPY --from=reactive_circumnav config src/reactive_circumnav/config
COPY --from=reactive_circumnav include src/reactive_circumnav/include
COPY --from=reactive_circumnav launch src/reactive_circumnav/launch
COPY --from=reactive_circumnav src src/reactive_circumnav/src
COPY --from=reactive_circumnav worlds src/reactive_circumnav/worlds

RUN source /opt/ros/jazzy/setup.bash \
  && colcon build \
       --merge-install \
       --packages-select sim_bot reactive_circumnav \
       --cmake-args -DBUILD_TESTING=OFF

COPY docker/entrypoint.sh /usr/local/bin/sim-bot-entrypoint
RUN chmod +x /usr/local/bin/sim-bot-entrypoint

ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/usr/local/bin/sim-bot-entrypoint"]
CMD ["ros2", "launch", "sim_bot", "sim_with_obstacles.launch.py"]
