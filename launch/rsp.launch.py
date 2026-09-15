import os
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


# Robot model name -> xacro entry point in description/.
ROBOT_DESCRIPTIONS = {
    'sim_bot': 'robot.urdf.xacro',
    'kobuki': 'kobuki.urdf.xacro',
}


def _robot_state_publisher(context):
    robot_model = LaunchConfiguration('robot_model').perform(context)

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('sim_bot'))
    xacro_file = os.path.join(pkg_path, 'description', ROBOT_DESCRIPTIONS[robot_model])
    profile = LaunchConfiguration('lidar_profile').perform(context)
    rate = LaunchConfiguration('lidar_update_rate').perform(context)
    if not math.isfinite(float(rate)) or float(rate) <= 0:
        raise ValueError('lidar_update_rate must be finite and positive')
    robot_description_config = xacro.process_file(
        xacro_file, mappings={'lidar_samples': '3601' if profile == 'mid360_2d' else '360',
                             'lidar_update_rate': rate})

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(),
              'use_sim_time': LaunchConfiguration('use_sim_time')}
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )]


def generate_launch_description():

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument('lidar_update_rate', default_value='10',
                              description='Simulated lidar update frequency in Hz'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'lidar_profile', default_value='ideal', choices=['ideal', 'mid360_2d'],
            description='Dense raw scan for the optional Mid-360 2D noise adapter'),

        DeclareLaunchArgument(
            'robot_model',
            default_value='sim_bot',
            choices=sorted(ROBOT_DESCRIPTIONS),
            description='Which robot description to publish'),

        OpaqueFunction(function=_robot_state_publisher)
    ])
