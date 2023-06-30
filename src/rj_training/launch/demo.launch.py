import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


# Entry point called by launch tool
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                parameters=[
                    os.path.join(
                        get_package_share_directory("rj_training"),
                        "config",
                        "turtlesim_params.yaml",
                    )
                ],
            ),
            Node(package="turtlesim", executable="draw_square"),
        ]
    )
