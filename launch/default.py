from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urmom',
            executable='tf2_broadcaster',
            name='broadcaster'
        ),        
        Node(
            package='urmom',
            executable='serialTest',
            name='serial',
        ),
        Node(
            package='urmom',
            executable='talker'
        ),
    ])