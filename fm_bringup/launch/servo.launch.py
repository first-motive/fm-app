"""MoveIt Servo — Cartesian + joint jogging, driven by the robot registry.

Brings up servo_node with the MoveIt context it needs: the robot_description (built
from the robot's backend-selectable xacro), an SRDF + kinematics + joint limits,
and servo.yaml. Servo subscribes /joint_states, turns delta twist / joint commands
into a streamed JointTrajectory, and publishes it to the arm's JTC.

The SRDF's joints must match the loaded model exactly, or the planning scene monitor
waits forever for the missing joints. So the description and the SRDF follow the same
robot + variant as the running sim. Each robot's MoveIt config locator, SRDF
selection, and servo.yaml live in fm_bringup.registry; this file holds none of it.

Teleop inputs publish onto servo_node/delta_twist_cmds and servo_node/delta_joint_cmds
(see teleop.launch.py). Started via the start_servo trigger below.
"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fm_bringup import registry


def _load_yaml(abs_path):
    with open(abs_path, "r") as handle:
        return yaml.safe_load(handle)


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    spec = registry.get(robot)
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    variant = LaunchConfiguration("variant").perform(context) or spec.default_variant

    # Description for the planning scene, built for the SAME variant as the sim so the
    # joint set matches. The <ros2_control> plugin is irrelevant to Servo; only
    # links/joints/collisions matter, so any backend parses fine.
    robot_description = spec.build_description(variant, sim_backend)
    robot_description_semantic = spec.semantic(variant)

    kinematics = _load_yaml(spec.moveit_file("kinematics.yaml"))
    joint_limits = _load_yaml(spec.moveit_file("joint_limits.yaml"))
    servo_yaml = _load_yaml(spec.servo_params_file())

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            {"moveit_servo": servo_yaml["moveit_servo"]},
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning": joint_limits},
        ],
    )

    # servo_node starts paused; trigger it once it is up.
    start_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    return [servo_node, start_servo]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="openarm",
                description="Robot to teleop (see fm_bringup.registry).",
            ),
            DeclareLaunchArgument(
                "sim_backend",
                default_value="mujoco",
                description="Backend the description is built for (parses under any).",
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description="Preset; must match the running sim. Empty uses the "
                "registry default.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
