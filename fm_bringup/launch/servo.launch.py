"""MoveIt Servo for the OpenArm — Cartesian + joint jogging of the right arm.

Brings up servo_node with the MoveIt context it needs: the robot_description (built
from the fm_control backend-selectable xacro), an SRDF + kinematics + joint limits,
and servo.yaml. Servo subscribes /joint_states, turns delta twist / joint commands
into a streamed JointTrajectory, and publishes it to the right arm's JTC.

The SRDF's joints must match the loaded model exactly, or the planning scene monitor
waits forever for the missing joints. So the description and the SRDF follow the same
variant as the running sim:

    right_arm         fm_bringup right_arm.srdf (single arm, 7 joints)
    default_bimanual  vendored openarm_bimanual.srdf (both arms + grippers)

kinematics + joint limits come from the vendored MoveIt config (its right_arm entry
applies to both). Teleop inputs publish onto servo_node/delta_twist_cmds and
servo_node/delta_joint_cmds (see teleop.launch.py). Started via the start_servo
trigger below.
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The vendored MoveIt config is bimanual v2.0; Servo drives its right_arm group.
_MOVEIT_PKG = "openarm_bimanual_moveit_config"
_MOVEIT_CFG = os.path.join("config", "openarm_v2.0")


def _load_yaml(abs_path):
    with open(abs_path, "r") as handle:
        return yaml.safe_load(handle)


def _moveit_file(name):
    return os.path.join(get_package_share_directory(_MOVEIT_PKG), _MOVEIT_CFG, name)


def _semantic(variant):
    """Read the SRDF matching the variant: single-arm for right_arm, else bimanual."""
    if variant == "right_arm":
        path = os.path.join(
            get_package_share_directory("fm_bringup"),
            "config",
            "openarm",
            "right_arm.srdf",
        )
    else:
        path = _moveit_file("openarm_bimanual.srdf")
    with open(path, "r") as handle:
        return handle.read()


def _launch_setup(context, *args, **kwargs):
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    variant = LaunchConfiguration("variant").perform(context) or "right_arm"

    # Description for the planning scene, built for the SAME variant as the sim so the
    # joint set matches. The <ros2_control> plugin is irrelevant to Servo; only
    # links/joints/collisions matter, so any backend parses fine.
    xacro_path = os.path.join(
        get_package_share_directory("fm_control"), "urdf", "openarm.sim.urdf.xacro"
    )
    robot_description = xacro.process_file(
        xacro_path, mappings={"robot_preset": variant, "sim_backend": sim_backend}
    ).toxml()

    robot_description_semantic = _semantic(variant)

    kinematics = _load_yaml(_moveit_file("kinematics.yaml"))
    joint_limits = _load_yaml(_moveit_file("joint_limits.yaml"))
    servo_yaml = _load_yaml(
        os.path.join(
            get_package_share_directory("fm_bringup"),
            "config",
            "openarm",
            "servo.yaml",
        )
    )

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
                "sim_backend",
                default_value="mujoco",
                description="Backend the description is built for (parses under any).",
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="right_arm",
                description="Preset; must match the running sim. right_arm or "
                "default_bimanual.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
