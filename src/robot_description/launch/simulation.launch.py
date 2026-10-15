import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Find our package
    package_path = get_package_share_directory(
        'robot_description'
    )

    # Xacro file
    xacro_file = os.path.join(
        package_path,
        'urdf',
        'robot.urdf.xacro'
    )

    # Gazebo world
    world_file = os.path.join(
        package_path,
        'worlds',
        'test_world.sdf'
    )

    # Convert Xacro to URDF
    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': world_file
        }.items()
    )

    # Publish robot TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'my_robot',
            '-z',
            '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
