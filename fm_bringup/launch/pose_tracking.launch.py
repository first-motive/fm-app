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
    # The pinch-gripper preset renames the flange (link7 -> ee_base_link); re-point every servo
    # frame that pinned the old flange or MoveIt Servo aborts at init ("Link ... not found").
    # ee_frame_name (servo.yaml) AND robot_link_command_frame (pose_tracking.yaml pins it to the
    # EE link) both reference it.
    ee_frame = spec.ee_frames.get(variant)
    if ee_frame:
        default_ee = spec.ee_frames.get(spec.default_variant)
        moveit_servo["ee_frame_name"] = ee_frame
        if moveit_servo.get("robot_link_command_frame") == default_ee:
            moveit_servo["robot_link_command_frame"] = ee_frame

    # Per-arm overrides for the BIMANUAL mirror: a second pose_tracking_node servos the OTHER
    # arm. Empty (single-arm) keeps servo.yaml. When set, this instance targets a different
    # planning group / EE / controller, and runs under `arm_ns` so its target_pose + ~/status
    # do not collide with the primary. pose_tracking pins robot_link_command_frame to the EE.
    arm_ns = LaunchConfiguration("arm_ns").perform(context)
    move_group = LaunchConfiguration("move_group").perform(context)
    ov_command_frame = LaunchConfiguration("command_frame").perform(context)
    ov_ee_frame = LaunchConfiguration("ee_frame").perform(context)
    ov_command_out = LaunchConfiguration("command_out").perform(context)
    if move_group:
        moveit_servo["move_group_name"] = move_group
    if ov_command_frame:
        moveit_servo["planning_frame"] = ov_command_frame
    if ov_ee_frame:
        moveit_servo["ee_frame_name"] = ov_ee_frame
        moveit_servo["robot_link_command_frame"] = ov_ee_frame   # pose tracking pins EE
    if ov_command_out:
        moveit_servo["command_out_topic"] = ov_command_out

    return [
        Node(
            package="fm_control",
            executable="pose_tracking_node",
            name="pose_tracking_node",
            namespace=arm_ns or None,
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
            # Per-arm overrides for the bimanual mirror (a second, namespaced pose_tracking).
            # All empty (default) -> single-arm behaviour straight from servo.yaml.
            DeclareLaunchArgument("arm_ns", default_value="",
                                  description="namespace for this arm's pose_tracking node"),
            DeclareLaunchArgument("move_group", default_value="",
                                  description="SRDF planning group (e.g. left_arm)"),
            DeclareLaunchArgument("command_frame", default_value="",
                                  description="Servo planning frame (arm base link)"),
            DeclareLaunchArgument("ee_frame", default_value="",
                                  description="arm tip link (Servo ee_frame_name + cmd frame)"),
            DeclareLaunchArgument("command_out", default_value="",
                                  description="this arm's joint_trajectory_controller topic"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
