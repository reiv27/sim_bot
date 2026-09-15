#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /opt/sim_bot_ws/install/setup.bash

exec "$@"
