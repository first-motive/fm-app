"""Unified OpenArm simulation launch — one control stack, swappable sim backend.

    robot + variant + sim_backend
        -> robot_description (fm_control backend-selectable xacro)
        -> robot_state_publisher + foxglove_bridge
        -> backend that hosts the controller_manager:
             mock / real   inline standalone ros2_control_node
             mujoco         mujoco_ros2_control (MuJoCo hosts the CM)
             gazebo         gz-sim (gz_ros2_control plugin hosts the CM)
             isaac          standalone ros2_control_node + Isaac topic bridge
        -> controller spawners (joint_state_broadcaster + arm/gripper controllers)

The controllers and the controller set are identical across backends; only the
<ros2_control> System plugin in the description swaps. Backend picks the compose
overlay in scripts/sim.sh (mock/mujoco -> macOS, gazebo/isaac -> Linux/GPU).
"""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Visual meshes ship as z-up .stl under fm_description (converted at build); the
# upstream xacro points visuals at openarm_description .dae. Rewrite so Foxglove
# (fed by robot_state_publisher) renders. Collisions stay on openarm_description.
_VISUAL_MESH_RE = re.compile(r"package://openarm_description/([^\"']+?)\.dae")
_VISUAL_MESH_SUB = r"package://fm_description/openarm_meshes/\1.stl"

# Backends that do not host their own controller_manager need a standalone one.
_STANDALONE_CM_BACKENDS = {"mock", "real"}

# Active + inactive controllers per variant. Names match the controllers.yaml and
# the description joint prefixes.
_CONTROLLERS = {
    "right_arm": {
        "active": ["openarm_right_arm_controller"],
        "inactive": ["openarm_right_forward_position_controller"],
    },
    "default_bimanual": {
        "active": [
            "openarm_left_arm_controller",
            "openarm_right_arm_controller",
            "openarm_left_gripper_controller",
            "openarm_right_gripper_controller",
        ],
        "inactive": [],
    },
}

_DEFAULT_VARIANT = "right_arm"


def _build_description(variant, sim_backend, controllers_file):
    """Process the fm_control backend-selectable xacro into a description string."""
    xacro_path = os.path.join(
        get_package_share_directory("fm_control"), "urdf", "openarm.sim.urdf.xacro"
    )
    mappings = {"robot_preset": variant, "sim_backend": sim_backend}
    # Gazebo's controller_manager lives in the description plugin, so it needs the
    # controllers file baked in.
    if sim_backend == "gazebo":
        mappings["gazebo_controllers_file"] = controllers_file
    doc = xacro.process_file(xacro_path, mappings=mappings)
    return _VISUAL_MESH_RE.sub(_VISUAL_MESH_SUB, doc.toxml())


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    variant = LaunchConfiguration("variant").perform(context) or _DEFAULT_VARIANT
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove")

    if robot != "openarm":
        raise RuntimeError(
            f"sim.launch.py supports robot:=openarm only (got '{robot}'). "
            "G1/SO101 are description-only for now."
        )
    if variant not in _CONTROLLERS:
        raise RuntimeError(
            f"No controllers.yaml for variant '{variant}'. "
            f"Available: {', '.join(sorted(_CONTROLLERS))}."
        )

    controllers_file = os.path.join(
        get_package_share_directory("fm_bringup"),
        "config",
        "openarm",
        f"{variant}.controllers.yaml",
    )
    robot_description = _build_description(variant, sim_backend, controllers_file)

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            parameters=[{"port": 8765, "address": "0.0.0.0"}],
            output="screen",
            condition=IfCondition(use_foxglove),
        ),
    ]

    # Backend that hosts the controller_manager.
    backends_dir = os.path.join(
        get_package_share_directory("fm_bringup"), "launch", "sim_backends"
    )
    if sim_backend in _STANDALONE_CM_BACKENDS:
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description},
                    controllers_file,
                ],
                output="screen",
            )
        )
    elif sim_backend in ("mujoco", "gazebo", "isaac"):
        # Gazebo spawns from the /robot_description topic (robot_state_publisher
        # above), so it needs no description passed; mujoco/isaac take it as a param.
        backend_args = {}
        if sim_backend in ("mujoco", "isaac"):
            backend_args["robot_description"] = robot_description
            backend_args["controllers_file"] = controllers_file
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(backends_dir, f"{sim_backend}.launch.py")
                ),
                launch_arguments=backend_args.items(),
            )
        )
    else:
        raise RuntimeError(f"Unknown sim_backend '{sim_backend}'.")

    # Controller spawners against whichever controller_manager came up above.
    spec = _CONTROLLERS[variant]
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("fm_bringup"),
                    "launch",
                    "controllers.launch.py",
                )
            ),
            launch_arguments={
                "controllers_file": controllers_file,
                "controllers": ",".join(spec["active"]),
                "inactive_controllers": ",".join(spec["inactive"]),
                "use_standalone_cm": "false",
            }.items(),
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="openarm",
                description="Robot to simulate (openarm only for now).",
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description="OpenArm preset; empty uses right_arm. "
                "One of: right_arm, default_bimanual.",
            ),
            DeclareLaunchArgument(
                "sim_backend",
                default_value="mujoco",
                description="mock | mujoco | gazebo | isaac | real.",
            ),
            DeclareLaunchArgument(
                "use_foxglove",
                default_value="true",
                description="Start foxglove_bridge on ws://0.0.0.0:8765.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
