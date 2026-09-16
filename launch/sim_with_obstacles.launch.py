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
      'robot_model': LaunchConfiguration('robot_model'),
      'lidar_profile': LaunchConfiguration('lidar_profile'),
      'lidar_noise_enabled': LaunchConfiguration('lidar_noise_enabled'),
      'lidar_update_rate': LaunchConfiguration('lidar_update_rate'),
      'lidar_noise_config': LaunchConfiguration('lidar_noise_config'),
      'spawn_x': LaunchConfiguration('spawn_x'),
      'spawn_y': LaunchConfiguration('spawn_y'),
      'spawn_yaw': LaunchConfiguration('spawn_yaw'),
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
    DeclareLaunchArgument('lidar_update_rate', default_value='10',
                         description='Simulated lidar update frequency in Hz'),
    DeclareLaunchArgument('lidar_profile', default_value='ideal',
                         choices=['ideal', 'mid360_2d']),
    DeclareLaunchArgument('lidar_noise_enabled', default_value='false',
                         choices=['true', 'false'],
                         description='Publish a noisy 360-ray scan on /scan'),
    DeclareLaunchArgument('lidar_noise_config', default_value=os.path.join(
        share, 'config', 'livox_mid360_noise.yaml')),
    DeclareLaunchArgument(
      'world',
      default_value=default_world,
      description='Gazebo world file (same as launch_sim.launch.py)',
    ),
    DeclareLaunchArgument(
      'robot_model',
      default_value='sim_bot',
      choices=['kobuki', 'sim_bot'],
      description='Robot description to simulate (same as launch_sim.launch.py)',
    ),
    DeclareLaunchArgument(
      'spawn_x', default_value='-3.0',
      description='Robot spawn X, metres (same as launch_sim.launch.py)',
    ),
    DeclareLaunchArgument(
      'spawn_y', default_value='0.0',
      description='Robot spawn Y, metres (same as launch_sim.launch.py)',
    ),
    DeclareLaunchArgument(
      'spawn_yaw', default_value='-1.5708',
      description='Robot spawn yaw, radians (same as launch_sim.launch.py)',
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
