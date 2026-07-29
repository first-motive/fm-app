"""Sample First Motive bringup.

Launches the foxglove_bridge (ws://0.0.0.0:8765 -> macOS Foxglove Studio) plus the
control node stub. Replace stubs as real nodes land.

``use_foxglove`` (default true) gates the bridge, so a run on a non-WebSocket
comms profile can bring the control stack up without it. Every other launch file
already carries this argument; this one started the bridge unconditionally.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fm_bringup.registry import DEFAULT_FOXGLOVE_PARAMS


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_foxglove",
                default_value="true",
                description="Start foxglove_bridge on ws://0.0.0.0:8765.",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                # The registry defaults every robot spec already carries: the same
                # port and address as before, plus the dotted-path mesh allowlist
                # and raised send buffer. Those two are additive — large meshes
                # that this bridge used to drop now render, matching sim.launch.py.
                parameters=[DEFAULT_FOXGLOVE_PARAMS],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_foxglove")),
            ),
            Node(
                package="fm_control",
                executable="control_node",
                name="control_node",
                output="screen",
            ),
        ]
    )
