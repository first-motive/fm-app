"""Unified simulation launch — one control stack, swappable robot + sim backend.

    robot + variant + sim_backend
        -> RobotSpec (fm_bringup.registry)
        -> robot_description (the robot's backend-selectable xacro)
        -> robot_state_publisher + foxglove_bridge
        -> backend that hosts the controller_manager:
             mock / real   inline standalone ros2_control_node
             mujoco         mujoco_ros2_control (MuJoCo hosts the CM)
             gazebo         gz-sim (gz_ros2_control plugin hosts the CM)
             isaac          standalone ros2_control_node + Isaac topic bridge
        -> controller spawners (joint_state_broadcaster + arm/gripper controllers)

The controllers and the controller set are identical across backends; only the
<ros2_control> System plugin in the description swaps. Each robot's specifics live
in fm_bringup.registry; this file holds no robot-specific data. Backend picks the
compose overlay in scripts/sim.sh (mock/mujoco -> macOS, gazebo/isaac -> Linux/GPU).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fm_bringup import registry


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    spec = registry.get(robot)
    variant = LaunchConfiguration("variant").perform(context) or spec.default_variant
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    task_env = LaunchConfiguration("task_env").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove")

    if variant not in spec.controllers:
        raise RuntimeError(
            f"No controllers.yaml for {robot} variant '{variant}'. "
            f"Available: {', '.join(sorted(spec.controllers))}."
        )

    controllers_file = spec.controllers_file(variant)
    mujoco_model = registry.resolve_task_env_mujoco_model(robot, task_env)
    robot_description = spec.build_description(
        variant,
        sim_backend,
        controllers_file,
        mujoco_model=mujoco_model,
    )

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
            parameters=[spec.foxglove_params],
            output="screen",
            condition=IfCondition(use_foxglove),
        ),
    ]

    # Robots whose controllers drive only a subset of the model (the G1-D arm) need a
    # joint_state_publisher to fill the unactuated joints, or Servo's planning scene
    # never completes. source_list takes the controlled joints from the broadcaster's
    # /joint_states; the rest publish at their URDF default.
    if spec.full_state_jsp:
        nodes.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "source_list": ["/joint_states"],
                        "rate": 30,
                    }
                ],
            )
        )

    # SO101 task environments live inside the MuJoCo MJCF only. Publish a matching
    # MarkerArray so Foxglove can render the table / props from the same task_env alias.
    if (
        robot == "so101"
        and sim_backend != "mujoco"
        and (task_env or "").strip() not in ("", "default")
    ):
        nodes.append(
            Node(
                package="fm_bringup",
                executable="task_env_markers",
                parameters=[{"robot": robot, "task_env": task_env}],
                output="screen",
            )
        )

    # Backend that hosts the controller_manager.
    backends_dir = os.path.join(
        get_package_share_directory("fm_sim_backends"), "launch"
    )
    if sim_backend in spec.standalone_cm_backends:
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description},
                    controllers_file,
                ],
                # Global remaps reach the controllers loaded into this manager (e.g.
                # diff_drive's cmd_vel_unstamped -> /cmd_vel). Empty for most robots.
                remappings=list(spec.cmd_remaps),
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
        if sim_backend == "mujoco":
            backend_args["robot"] = robot
            backend_args["task_env"] = task_env
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
    cset = spec.controllers[variant]
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
                "controllers": ",".join(cset["active"]),
                "inactive_controllers": ",".join(cset["inactive"]),
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
                description="Robot to simulate (see fm_bringup.registry).",
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description="Robot preset; empty uses the registry default.",
            ),
            DeclareLaunchArgument(
                "sim_backend",
                default_value="mujoco",
                description="mock | mujoco | gazebo | isaac | real.",
            ),
            DeclareLaunchArgument(
                "task_env",
                default_value="default",
                description="Named task environment alias. so101 supports: default | table_reach | pick_place | bin_sort.",
            ),
            DeclareLaunchArgument(
                "use_foxglove",
                default_value="true",
                description="Start foxglove_bridge on ws://0.0.0.0:8765.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
