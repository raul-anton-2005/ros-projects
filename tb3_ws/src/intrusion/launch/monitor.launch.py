from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="obstacles",
                executable="detector",
                namespace="north",
                parameters=[
                    {"obs_angle_min": -0.3927},  # -22.5°
                    {"obs_angle_max": 0.3927},  # +22.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                package="obstacles",
                executable="detector",
                namespace="northwest",
                parameters=[
                    {"obs_angle_min": 0.3927},  # +22.5°
                    {"obs_angle_max": 1.1781},  # +67.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                package="obstacles",
                executable="detector",
                namespace="west",
                parameters=[
                    {"obs_angle_min": 1.1781},  # +67.5°
                    {"obs_angle_max": 1.9635},  # +112.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                package="obstacles",
                executable="detector",
                namespace="southwest",
                parameters=[
                    {"obs_angle_min": 1.9635},  # +112.5°
                    {"obs_angle_max": 2.7490},  # +157.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                # Dividir en 2 partes de -157.5º a 360º y de -360º a -157.5º
                package="obstacles",
                executable="detector",
                namespace="south_1",
                parameters=[
                    {"obs_angle_min": 2.7490},  # -157.5°
                    {"obs_angle_max": 3.1415},  # -180°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                package="obstacles",
                executable="detector",
                namespace="south_2",
                parameters=[
                    {"obs_angle_min": -3.1415},  # -180°
                    {"obs_angle_max": -2.7490},  # -157.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                package="obstacles",
                executable="detector",
                namespace="southeast",
                parameters=[
                    {"obs_angle_min": -2.7490},  # -157.5°
                    {"obs_angle_max": -1.9635},  # -112.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                package="obstacles",
                executable="detector",
                namespace="east",
                parameters=[
                    {"obs_angle_min": -1.9635},  # -112.5°
                    {"obs_angle_max": -1.1781},  # -67.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(
                package="obstacles",
                executable="detector",
                namespace="northeast",
                parameters=[
                    {"obs_angle_min": -1.1781},  # -67.5°
                    {"obs_angle_max": -0.3927},  # -22.5°
                    {"obs_threshold": 2.0},
                ],
            ),
            Node(package="intrusion", executable="monitor", output="screen"),
        ]
    )
