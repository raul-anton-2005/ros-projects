from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_trajectory',
            executable='polygon',
            parameters=[{"linear_speed": 0.8, "angular_speed": 0.5, "segment_size": 1.25, "number_segments": 6}],
        )
        # Add more nodes here but don't forget to add a comma at the end of the previous line
    ])