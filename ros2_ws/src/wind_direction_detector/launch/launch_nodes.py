from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wind_direction_detector',
            executable='random_move',
            name='random_move'
        )
    ])