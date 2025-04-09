from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                package='ros2_project_sc22as2',
                executable='robot',
                name='Robot'
            ),
            Node(
                package='ros2_project_sc22as2',
                executable='navigation',
                name='Navigation'
            ),
        ])
