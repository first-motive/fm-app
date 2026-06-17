"""Sample First Motive bringup.

Launches the foxglove_bridge (ws://0.0.0.0:8765 -> macOS Foxglove Studio) plus the
control node stub. Replace stubs as real nodes land.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                parameters=[{"port": 8765, "address": "0.0.0.0"}],
                output="screen",
            ),
            Node(
                package="fm_control",
                executable="control_node",
                name="control_node",
                output="screen",
            ),
        ]
    )
