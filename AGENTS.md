# AGENTS.md

## Project
This package is for simulation of mobile diff-drive robot in Gazebo

## Workflow
Before changing code:
- inspect the target file;
- inspect its callers;
- inspect relevant tests.

When changing code:
- make the smallest change that solves the task;
- do not modify unrelated code;
- preserve existing interfaces unless necessary.

After changing code:
- build the affected package;
- run relevant tests;
- inspect failures before making additional changes.

## Robotics rules
- Use SI units.
- Do not change coordinate-frame or sign conventions implicitly.

## Research rules
- Do not claim an improvement without comparing metrics.
- Change one conceptual variable at a time when testing a hypothesis.

## Repo tree
sim_bot/
├── config/         # ROS, Nav2, Gazebo, RViz and sensor configs
├── description/    # URDF/Xacro robot description
├── launch/         # ROS 2 launch files
├── meshes/         # Robot 3D models and textures
├── plugins/        # Gazebo/C++ plugins
├── scripts/        # Python nodes and utilities
├── test/           # Unit/integration tests
└── worlds/         # Gazebo worlds
