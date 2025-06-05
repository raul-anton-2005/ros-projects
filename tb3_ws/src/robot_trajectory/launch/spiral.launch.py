from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_trajectory",
                executable="spiral",
                remappings=[
                    ("/cmd_vel", "/turtle1/cmd_vel"),
                ],
                parameters=[{"angular_speed": 6.0}, {"distance_between_loops": 0.5}, {"number_of_loops": 6}],
            ),
            Node(package="turtlesim", executable="turtlesim_node"),
        ]
    )
