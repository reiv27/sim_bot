# Docker environment

This directory runs `sim_bot` in Ubuntu 24.04 with ROS 2 Jazzy and Gazebo
Harmonic. The host only needs Docker, an X11-compatible display and a working
graphics driver.

Run all commands from this directory:

```bash
./build.sh
./run.sh
```

`run.sh` starts the container in the background and launches the robot together
with the moving obstacles through:

```bash
ros2 launch sim_bot sim_with_obstacles.launch.py
```

ROS launch arguments can be passed directly to `run.sh` or `restart.sh`:

```bash
./run.sh robot_model:=kobuki lidar_profile:=mid360_2d
```

To use the rigid obstacle formation included in the package:

```bash
./run.sh \
  robot_model:=kobuki \
  obstacles_config:=/opt/sim_bot_ws/install/share/sim_bot/config/obstacles_rigid_formation.yaml
```

To use the static three-body layout from the reference image:

```bash
./run.sh \
  obstacles_config:=/opt/sim_bot_ws/install/share/sim_bot/config/obstacles_static_layout.yaml
```

Container lifecycle:

```bash
./exec.sh                  # interactive shell with ROS environment sourced
./exec.sh ros2 topic list  # execute one command in the container
./restart.sh               # recreate and restart the simulation
./stop.sh                  # stop and remove the container
```

The image and container names can be overridden without editing the scripts:

```bash
SIM_BOT_IMAGE=my-sim:dev ./build.sh
SIM_BOT_IMAGE=my-sim:dev SIM_BOT_CONTAINER=my-sim ./run.sh
```

The scripts pass the host X11 socket, available GPU devices and `/dev/input`
into the container. If Gazebo reports an X11 authorization error, make sure the
host `XAUTHORITY` variable points to a readable authority file before running
`./run.sh`.
