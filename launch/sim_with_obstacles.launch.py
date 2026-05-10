"""
Full simulation: robot (Gazebo + bridge + RSP + joystick) and moving obstacles.

  ros2 launch sim_bot sim_with_obstacles.launch.py

Obstacles start after ``obstacles_start_delay`` seconds so Gazebo can finish
initialising (see obstacles.launch.py). Override world or obstacle config:

  ros2 launch sim_bot sim_with_obstacles.launch.py \\
    world:=/path/to/world.sdf \\
    obstacles_config:=/path/to/obstacles.yaml \\
    obstacles_start_delay:=10.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  pkg = 'sim_bot'
  share = get_package_share_directory(pkg)
  default_world = os.path.join(share, 'worlds', 'empty.world')
  default_obstacles_cfg = os.path.join(share, 'config', 'obstacles.yaml')

  sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(share, 'launch', 'launch_sim.launch.py')
    ),
    launch_arguments={
      'world': LaunchConfiguration('world'),
    }.items(),
  )

  obstacles = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(share, 'launch', 'obstacles.launch.py')
    ),
    launch_arguments={
      'obstacles_config': LaunchConfiguration('obstacles_config'),
    }.items(),
  )

  obstacles_delayed = TimerAction(
    period=LaunchConfiguration('obstacles_start_delay'),
    actions=[obstacles],
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'world',
      default_value=default_world,
      description='Gazebo world file (same as launch_sim.launch.py)',
    ),
    DeclareLaunchArgument(
      'obstacles_config',
      default_value=default_obstacles_cfg,
      description='YAML for moving obstacles (obstacles.launch.py)',
    ),
    DeclareLaunchArgument(
      'obstacles_start_delay',
      default_value='8.0',
      description=(
        'Delay in seconds after starting sim before launching obstacles '
        '(spawn/bridge/controller use additional internal delays).'
      ),
    ),
    sim,
    obstacles_delayed,
  ])
