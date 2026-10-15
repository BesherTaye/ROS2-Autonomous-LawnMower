import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Find the robot_description package
    package_path = get_package_share_directory('robot_description')

    # Find our Xacro file
    xacro_file = os.path.join(
        package_path,
        'urdf',
        'robot.urdf.xacro'
    )

    # Convert Xacro → URDF
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        # Publishes TF transforms based on the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description}
            ]
        ),

        # Publishes joint states
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Opens RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
