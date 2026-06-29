"""MoveIt Servo PoseTracking — absolute EE pose servoing for the vision teleop path.

Launches fm_control/pose_tracking_node, which hosts moveit_servo::PoseTracking. It REPLACES
servo_node_main on the vision path: PoseTracking embeds its own Servo instance, so running both
against one controller would mean two writers to the arm's JointTrajectory controller. The
vision source publishes an absolute EE target pose (/target_pose) and this node servos the EE to
it and holds — true position mirroring, not twist jogging.

Built from the SAME registry context as servo.launch.py (robot_description + SRDF + kinematics +
joint limits + servo.yaml), plus pose_tracking.yaml deltas (command_in_type: speed_units and the
PoseTracking PID gains) deep-merged into the moveit_servo namespace. No start_servo trigger —
PoseTracking starts its embedded Servo in its constructor.
"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    # Same MoveIt context servo.launch.py builds, for the SAME variant as the running sim so the
    # joint set matches the planning-scene monitor's expectations.
    #
    # NOTE: we deliberately DO NOT pass robot_description_kinematics. With a kinematics solver
    # configured, moveit_servo computes joint deltas via searchPositionIK (return_approximate_
    # solution=true); for the redundant 7-DOF OpenArm that IK branch drifts toward the straight-
    # elbow solution (the singularity) even at near-zero error. With NO solver, moveit_servo uses
    # the inverse-Jacobian (minimum-norm) path, which stays near the current configuration and
    # does not posture-drift. This is the fix for the "drifts to straight" behavior.
    robot_description = spec.build_description(variant, sim_backend)
    robot_description_semantic = spec.semantic(variant)
    joint_limits = _load_yaml(spec.moveit_file("joint_limits.yaml"))

    # servo.yaml (frames, scale, singularity thresholds, command_out_topic) + pose_tracking.yaml
    # deltas (speed_units + PID gains), deep-merged so PoseTracking reads everything — including
    # its *_proportional_gain PID params — from the moveit_servo namespace.
    servo_yaml = _load_yaml(spec.servo_params_file())
    pose_yaml = _load_yaml(spec.pose_tracking_params_file())
    moveit_servo = dict(servo_yaml["moveit_servo"])
    moveit_servo.update(pose_yaml.get("moveit_servo", {}))

    return [
        Node(
            package="fm_control",
            executable="pose_tracking_node",
            name="pose_tracking_node",
            output="screen",
            parameters=[
                {"moveit_servo": moveit_servo},
                {"robot_description": robot_description},
                {"robot_description_semantic": robot_description_semantic},
                # robot_description_kinematics intentionally omitted -> inverse-Jacobian servoing
                # (see note above): avoids the redundant-arm IK drift to the straight singularity.
                {"robot_description_planning": joint_limits},
            ],
        )
    ]


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
