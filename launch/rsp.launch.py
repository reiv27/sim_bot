import os

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
    robot_description_config = xacro.process_file(xacro_file)

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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'robot_model',
            default_value='sim_bot',
            choices=sorted(ROBOT_DESCRIPTIONS),
            description='Which robot description to publish'),

        OpaqueFunction(function=_robot_state_publisher)
    ])
