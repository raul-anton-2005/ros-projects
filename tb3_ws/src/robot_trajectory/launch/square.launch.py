from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_trajectory',
            executable='square',
            remappings=[('/cmd_vel', '/turtle1/cmd_vel')],
            parameters=[{"linear_speed": 3.0, "angular_speed": 1.5, "square_length": 4.0}],
        )
        # Add more nodes here but don't forget to add a comma at the end of the previous line
    ])
