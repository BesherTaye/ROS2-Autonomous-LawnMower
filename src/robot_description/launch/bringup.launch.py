import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    robot_description_share = get_package_share_directory(
        'robot_description'
    )

    # Existing launch files
    simulation_launch = os.path.join(
        robot_description_share,
        'launch',
        'simulation.launch.py'
    )

    ekf_launch = os.path.join(
        robot_description_share,
        'launch',
        'ekf.launch.py'
    )

    amcl_launch = os.path.join(
        robot_description_share,
        'launch',
        'amcl.launch.py'
    )

    nav2_launch = os.path.join(
        robot_description_share,
        'launch',
        'nav2.launch.py'
    )

    # Configuration files
    bridge_config = os.path.join(
        robot_description_share,
        'config',
        'bridge.yaml'
    )

    rviz_config = os.path.join(
        robot_description_share,
        'rviz',
        'navigation.rviz'
    )


    # =========================================================
    # SOFTWARE RENDERING
    # =========================================================
    #
    # This is required on your machine because Gazebo rendering
    # was unstable with normal GPU rendering.

    software_rendering = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1'
    )


    # =========================================================
    # GAZEBO SIMULATION
    # =========================================================

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            simulation_launch
        )
    )


    # =========================================================
    # ROS <-> GAZEBO BRIDGE
    # =========================================================

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config
            }
        ]
    )


    # =========================================================
    # EKF
    # =========================================================
    #
    # EKF owns:
    #
    #     odom -> base_link
    #

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ekf_launch
        )
    )


    # =========================================================
    # AMCL + MAP SERVER
    # =========================================================
    #
    # AMCL owns:
    #
    #     map -> odom
    #

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            amcl_launch
        )
    )


    # =========================================================
    # NAV2
    # =========================================================

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_launch
        )
    )


    # =========================================================
    # RVIZ
    # =========================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ],
        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )


    # =========================================================
    # STARTUP ORDER
    # =========================================================
    #
    # Some components need TF and /clock to exist before they
    # activate. Small delays make startup much more reliable.
    #
    # 0 s  Gazebo
    # 1 s  bridge
    # 3 s  EKF
    # 5 s  AMCL/map_server
    # 8 s  Nav2
    # 10 s RViz

    delayed_bridge = TimerAction(
        period=1.0,
        actions=[bridge]
    )

    delayed_ekf = TimerAction(
        period=3.0,
        actions=[ekf]
    )

    delayed_localization = TimerAction(
        period=5.0,
        actions=[localization]
    )

    delayed_navigation = TimerAction(
        period=8.0,
        actions=[navigation]
    )

    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz]
    )


    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([
        software_rendering,

        simulation,

        delayed_bridge,
        delayed_ekf,
        delayed_localization,
        delayed_navigation,
        delayed_rviz,
    ])