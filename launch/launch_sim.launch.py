import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from launch_ros.actions import Node


# Drop height per model: enough to clear the ground plane, not enough to bounce.
ROBOT_SPAWN_Z = {
    'sim_bot': '0.1',
    'kobuki': '0.05',
}


def _spawn_entity(context):
    # Run the spawner node from the ros_gz_sim package.
    # The entity name doesn't really matter if you only have a single robot.
    robot_model = LaunchConfiguration('robot_model').perform(context)
    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', LaunchConfiguration('robot_name'),
                   '-x', '-3.0',
                   '-y', '0.0',
                   '-z', ROBOT_SPAWN_Z[robot_model],
                   '-Y', '-1.5708'],
        output='screen',
    )]


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name = 'sim_bot'

    robot_model = LaunchConfiguration('robot_model')

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='sim_bot',
        choices=sorted(ROBOT_SPAWN_Z),
        description='Robot description to simulate (see rsp.launch.py)'
    )

    # The world's TrajectoryVisualPlugin tracks this name, so it stays 'my_bot'
    # whichever model is spawned.
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_bot',
        description='Entity name for the spawned robot in Gazebo'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true',
                                      'use_ros2_control': 'false',
                                      'robot_model': robot_model}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Let Gazebo resolve package://sim_bot/meshes/... from the install space.
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''), ':',
         os.path.dirname(get_package_share_directory(package_name))]
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    spawn_entity = OpaqueFunction(function=_spawn_entity)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    return LaunchDescription([
        robot_model_arg,
        robot_name_arg,
        gz_resource_path,
        rsp,
        joystick,
        gazebo,
        spawn_entity,
        world_arg,
        ros_gz_bridge,
        ros_gz_image_bridge,
        #diff_drive_spawner,  # Uncommit for ros2_control
        #joint_broad_spawner  # Uncommit for ros2_control
        # ultrasonic_data,
        #fake_controller,
        #neuromorphic_controller,
        # pose_listener,
        #path_publisher,
    ])
