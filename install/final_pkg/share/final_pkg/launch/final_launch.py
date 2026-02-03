from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_pkg',
            executable='node_first',
            name='servoJ_client',
            output='screen'
        ),
        Node(
            package='final_pkg',
            executable='read_from_json',
            name='read_from_json',
            output='screen'
        )
    ])
