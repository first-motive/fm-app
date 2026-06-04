"""Controller bringup for the OpenArm ros2_control stack.

Reusable across every sim backend. Two responsibilities:

  1. Optionally start a standalone controller_manager (``ros2_control_node``). The
     mock and real backends need this; mujoco/gazebo/isaac embed their own
     controller_manager inside the sim process, so they pass
     ``use_standalone_cm:=false`` and only spawn controllers.
  2. Spawn joint_state_broadcaster plus the requested controllers against the
     controller_manager (active), and any inactive controllers with ``--inactive``.

The controller set is identical across backends — only the System plugin in the
description swaps. ``controllers`` and ``inactive_controllers`` are comma-separated
so ``sim.launch.py`` can pick them per preset.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _split(value):
    """Split a comma-separated launch arg into a clean list of names."""
    return [item.strip() for item in value.split(",") if item.strip()]


def _spawner(controller, controller_manager, controllers_file, inactive=False):
    """A controller_manager spawner node for one controller."""
    args = [controller, "--controller-manager", controller_manager]
    if controllers_file:
        args += ["--param-file", controllers_file]
    if inactive:
        args += ["--inactive"]
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    controller_manager = LaunchConfiguration("controller_manager").perform(context)
    controllers = _split(LaunchConfiguration("controllers").perform(context))
    inactive = _split(LaunchConfiguration("inactive_controllers").perform(context))
    use_standalone_cm = LaunchConfiguration("use_standalone_cm")
    robot_description = LaunchConfiguration("robot_description").perform(context)

    nodes = []

    # Standalone controller_manager for backends that do not host their own.
    nodes.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controllers_file,
            ],
            output="screen",
            condition=IfCondition(use_standalone_cm),
        )
    )

    # joint_state_broadcaster first, then the active controllers, then inactive.
    nodes.append(_spawner("joint_state_broadcaster", controller_manager, controllers_file))
    for controller in controllers:
        nodes.append(_spawner(controller, controller_manager, controllers_file))
    for controller in inactive:
        nodes.append(
            _spawner(controller, controller_manager, controllers_file, inactive=True)
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                description="Path to the controllers.yaml for the active preset.",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Controller manager node name to spawn against.",
            ),
            DeclareLaunchArgument(
                "controllers",
                default_value="",
                description="Comma-separated controllers to spawn active "
                "(e.g. openarm_right_arm_controller).",
            ),
            DeclareLaunchArgument(
                "inactive_controllers",
                default_value="",
                description="Comma-separated controllers to load inactive.",
            ),
            DeclareLaunchArgument(
                "use_standalone_cm",
                default_value="false",
                description="Start a standalone ros2_control_node. True for mock/real; "
                "false for sim backends that host their own controller_manager.",
            ),
            DeclareLaunchArgument(
                "robot_description",
                default_value="",
                description="Robot description XML, used only when "
                "use_standalone_cm is true.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
