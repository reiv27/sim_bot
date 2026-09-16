# Docker environment

This directory builds `sim_bot` and the adjacent `reactive_circumnav` package
in Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. The host only needs
Docker, an X11-compatible display and a working graphics driver.

Run all commands from this directory:

```bash
./build.sh
./run.sh
```

By default, `build.sh` expects the algorithm source at
`../../reactive_circumnav` relative to this directory. Override it when the
repository is elsewhere:

```bash
REACTIVE_CIRCUMNAV_SOURCE=/path/to/reactive_circumnav ./build.sh
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
  spawn_yaw:=0 \
  obstacles_config:=/opt/sim_bot_ws/install/share/sim_bot/config/obstacles_static_layout.yaml
```

After the simulation has started, run the controller in a second terminal:

```bash
./algorithm.sh
```

The controller runs inside the existing simulation container, so it uses the
same ROS 2 Jazzy installation, DDS implementation and ROS domain as the
Gazebo bridge. Press `Ctrl+C` in the second terminal to stop the controller.
Each run creates two timestamped files in the host directory
`docker/telemetry/`:

- `reactive_circumnav_*.csv` contains controller telemetry;
- `reactive_circumnav_timing_*.csv` contains callback timing telemetry.

The directory is mounted in the container at
`/opt/sim_bot_data/telemetry`. Override the host directory by passing the same
absolute path to both simulation and controller commands:

```bash
SIM_BOT_TELEMETRY_DIR=/path/on/host ./restart.sh
SIM_BOT_TELEMETRY_DIR=/path/on/host ./algorithm.sh
```

The controller parameter file can still be overridden with
`REACTIVE_CIRCUMNAV_PARAMS_FILE`; its value must be a path visible inside the
container.

Container lifecycle:

```bash
./exec.sh                  # interactive shell with ROS environment sourced
./exec.sh ros2 topic list  # execute one command in the container
./algorithm.sh             # run reactive_circumnav in the container
./restart.sh               # recreate and restart the simulation
./stop.sh                  # stop and remove the container
```

The image and container names can be overridden without editing the scripts:

```bash
SIM_BOT_IMAGE=my-sim:dev ./build.sh
SIM_BOT_IMAGE=my-sim:dev SIM_BOT_CONTAINER=my-sim ./run.sh
```

Each container uses its container name as a separate Gazebo transport partition,
so two Gazebo worlds cannot accidentally discover each other. Override it, or
optionally select a ROS domain, with `SIM_BOT_GZ_PARTITION` and
`SIM_BOT_ROS_DOMAIN_ID`.

The scripts pass the host X11 socket, available GPU devices and `/dev/input`
into the container. If Gazebo reports an X11 authorization error, make sure the
host `XAUTHORITY` variable points to a readable authority file before running
`./run.sh`.
