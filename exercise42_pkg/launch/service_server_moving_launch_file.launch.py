from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise42_pkg',
            executable='service_server_moving_right',
            output='screen'),
    ])