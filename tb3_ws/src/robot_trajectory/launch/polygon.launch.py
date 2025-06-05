from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_trajectory',
            executable='polygon',
            remappings=[('/cmd_vel', '/turtle1/cmd_vel')],
            parameters=[{"linear_speed": 1.0, "angular_speed": 1.2, "side_length": 3.0, "n_sides": 5}],
        )
        # Add more nodes here but don't forget to add a comma at the end of the previous line
    ])
