"""Remote-sourced mirror teleop — the sim robot mirrors a hand tracked on a REMOTE rig.

The recorder rig (a headless Linux/Jetson host) runs the camera + ``hand_tracker`` and publishes
the hand streams over ROS2 DDS: ``/vision/hand_pose`` (+ ``/vision/grip``,
``/vision/tracking_active``, ``/vision/engage``, ``/vision/image``). This launch brings up ONLY
the robot side on the operator's Mac — sim + pose_tracking + a bare ``mirror_source`` that
subscribes those streams over the network — so the sim arm mirrors the operator's hand with **no
local camera or MediaPipe**. Contrast ``vision_session.launch.py``, which runs the camera + tracker
locally; here the tracker lives on the rig and only tiny pose messages cross the wire.

    sim.launch.py          robot_state_publisher (/robot_description) + foxglove_bridge (:8765)
                           + the sim backend + controllers
    pose_tracking.launch   MoveIt Servo PoseTracking (embeds its own Servo, owns the arm's JTC),
                           held back by ``teleop_delay`` so the sim's /joint_states exists first
    mirror_source          subscribes the rig's /vision/hand_pose -> /target_pose (absolute EE)
    arm_reset              /vision/reset -> disengage + drive the arm home (Foxglove RESET)
    capture_browser        serve recorded sessions to the web GUI

The Mac must be joined to the rig's DDS graph (native pixi + ``scripts/run/dds-lan.sh``;
``./run.sh --native`` does this) so the ``/vision/*`` topics actually arrive. Single-arm
(openarm ``right_arm``) — the rig tracks one hand. Recording lives rig-side (the TUI's Record
action publishes ``/capture/record``), so there is no local datalogger here. By default
(``depth:=on``) the mirror is driven by the rig's **metric depth-Z** (``/vision/hand_point``) via
mirror_source's ``metric`` input mode; ``depth:=off`` falls back to the mono ``/vision/hand_pose``.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fm_bringup import registry


def _include(name, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fm_bringup"), "launch", name)
        ),
        launch_arguments=launch_arguments.items(),
    )


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    variant = LaunchConfiguration("variant").perform(context)
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove").perform(context)
    teleop_delay = float(LaunchConfiguration("teleop_delay").perform(context))
    depth_on = LaunchConfiguration("depth").perform(context).strip().lower() in ("on", "true", "1", "yes")
    want_gripper = LaunchConfiguration("gripper").perform(context).strip().lower() in ("on", "true", "1", "yes")

    # gripper:=on upgrades the base right_arm to the pinch-gripper preset (URDF gripper + the
    # openarm_right_gripper_controller). default_bimanual already carries grippers on both arms,
    # so it is left as-is — gripper then only toggles whether they are ACTUATED (§6).
    if want_gripper and variant in ("", "right_arm"):
        variant = "right_arm_with_pinch_gripper"

    spec = registry.get(robot)
    resolved_variant = variant or spec.default_variant
    # Per-arm mirror config when the variant drives both arms (default_bimanual); None single-arm.
    bimanual_arms = spec.mirror_arms.get(resolved_variant)

    # 1) Sim owns /robot_description, robot_state_publisher, foxglove_bridge, and the controllers.
    sim = _include(
        "sim.launch.py",
        {
            "robot": robot,
            "variant": variant,
            "sim_backend": sim_backend,
            "use_foxglove": use_foxglove,
        },
    )

    # mirror_source params load from the robot's vision.yaml; the target is stamped in the Servo
    # command frame (read from the same servo.yaml Servo loads, so they never drift).
    vision_yaml = spec.vision_params_file()
    base_params = [vision_yaml] if os.path.exists(vision_yaml) else []
    servo_yaml = spec.servo_params_file()
    try:
        with open(servo_yaml) as servo_file:
            servo_cfg = yaml.safe_load(servo_file)
        command_frame = servo_cfg["moveit_servo"]["robot_link_command_frame"]
    except (OSError, KeyError, TypeError) as exc:
        raise RuntimeError(
            f"Could not read robot_link_command_frame from {servo_yaml} for robot "
            f"'{robot}': {exc}. The mirror target must be stamped in that frame."
        ) from exc

    def _pose_tracking(extra=None):
        args = {"robot": robot, "sim_backend": sim_backend, "variant": variant}
        args.update(extra or {})
        return _include("pose_tracking.launch.py", args)

    def _rig_mirror_source(name, hand, overrides):
        # A mirror_source fed from a REMOTE rig hand stream — NO local camera/tracker.
        # depth:=on drives from the rig's per-hand metric depth /vision/<hand>/hand_point
        # (real RealSense Z, axis_gain[2]=1); depth:=off from the mono /vision/<hand>/hand_pose.
        params = dict(overrides)
        params["grip_topic"] = "vision/%s/grip" % hand
        params["tracking_topic"] = "vision/%s/tracking_active" % hand
        if depth_on:
            params["input_mode"] = "metric"
            params["hand_point_topic"] = "/vision/%s/hand_point" % hand
            params["axis_gain"] = [1.0, 1.0, 1.0]
        else:
            params["hand_pose_topic"] = "vision/%s/hand_pose" % hand
        return Node(
            package="fm_teleop_vision", executable="mirror_source",
            name=name, output="screen", parameters=base_params + [params])

    def _gripper_teleop(name, g):
        return Node(
            package="fm_teleop_device", executable="gripper_teleop", name=name, output="screen",
            parameters=[{
                "preset_topic": g["preset_topic"], "command_topic": g["command_topic"],
                "joints": list(g["joints"]), "open_positions": list(g["open"]),
                "close_positions": list(g["close"]),
            }])

    capture_browser = Node(
        package="fm_teleop_vision", executable="capture_browser",
        name="capture_browser", output="screen",
    )

    # --- BIMANUAL: one pose_tracking + mirror_source + arm_reset (+ gripper) per arm, each fed
    # its own hand's rig streams (left hand -> left arm, right hand -> right arm). ---
    if bimanual_arms:
        # 2) PoseTracking per arm, held back by teleop_delay so the sim comes up first.
        pose_tracking = TimerAction(period=teleop_delay, actions=[
            _pose_tracking({
                "arm_ns": arm["ns"], "move_group": arm["move_group"],
                "command_frame": arm["command_frame"], "ee_frame": arm["ee_frame"],
                "command_out": arm["command_out"],
            }) for arm in bimanual_arms
        ])
        mirror_nodes, reset_nodes, gripper_nodes = [], [], []
        for arm in bimanual_arms:
            over = {
                "command_frame": arm["command_frame"], "ee_frame": arm["ee_frame"],
                "target_pose_topic": "/%s/target_pose" % arm["ns"],
                "gripper_preset_topic": arm["gripper_preset"],
            }
            for key in ("workspace_min", "workspace_max", "axis_map_linear", "mirror_gain"):
                if key in arm:
                    over[key] = arm[key]
            mirror_nodes.append(_rig_mirror_source("mirror_source_%s" % arm["ns"], arm["hand"], over))
            reset_nodes.append(Node(
                package="fm_teleop_vision", executable="arm_reset",
                name="arm_reset_%s" % arm["ns"], output="screen",
                parameters=[{
                    "target_pose_topic": "/%s/target_pose" % arm["ns"],
                    "command_frame": arm["command_frame"],
                    "home_ee_position": list(arm["reset_home_ee"]["position"]),
                    "home_ee_orientation": list(arm["reset_home_ee"]["orientation"]),
                }]))
            # 6) Actuate the pinchers only when gripper:=on (else grippers stay in the model, idle).
            if want_gripper and arm.get("gripper"):
                gripper_nodes.append(_gripper_teleop("gripper_teleop_%s" % arm["ns"], arm["gripper"]))
        # Sag-seed both arms to their ready pose before commands (mujoco has real gravity).
        seed_nodes = []
        if sim_backend != "real":
            seed_actions = []
            for arm in bimanual_arms:
                rj = arm.get("ready_joints")
                if not rj:
                    continue
                names = ",".join("openarm_%s_joint%d" % (arm["ns"], j) for j in range(1, 8))
                positions = ",".join(str(v) for v in rj)
                msg = ("{joint_names: [%s], points: [{positions: [%s], "
                       "time_from_start: {sec: 1}}]}" % (names, positions))
                seed_actions.append(ExecuteProcess(
                    cmd=["ros2", "topic", "pub", "-t", "4", "-r", "2",
                         "/openarm_%s_arm_controller/joint_trajectory" % arm["ns"],
                         "trajectory_msgs/msg/JointTrajectory", msg], output="screen"))
            if seed_actions:
                seed_nodes = [TimerAction(period=max(4.0, teleop_delay - 4.0), actions=seed_actions)]
        return [sim, pose_tracking, *mirror_nodes, *reset_nodes, *gripper_nodes, *seed_nodes, capture_browser]

    # --- SINGLE ARM: the rig's one control hand (/vision/hand_point + /vision/hand_pose). ---
    pose_tracking = TimerAction(period=teleop_delay, actions=[_pose_tracking()])
    over = {"command_frame": command_frame}
    ee_frame = spec.ee_frames.get(resolved_variant)
    if ee_frame:
        over["ee_frame"] = ee_frame
    # single-arm reads the non-namespaced control streams (the rig's selected control hand).
    if depth_on:
        over["input_mode"] = "metric"
        over["hand_point_topic"] = "/vision/hand_point"
        over["axis_gain"] = [1.0, 1.0, 1.0]
    else:
        over["hand_pose_topic"] = "/vision/hand_pose"
    mirror_source = Node(
        package="fm_teleop_vision", executable="mirror_source", name="mirror_source",
        output="screen", parameters=base_params + [over])
    arm_reset = Node(
        package="fm_teleop_vision", executable="arm_reset", name="arm_reset",
        output="screen", parameters=[{"command_frame": command_frame}])
    nodes = [sim, pose_tracking, mirror_source, arm_reset]
    if want_gripper and spec.gripper:
        nodes.append(_gripper_teleop("gripper_teleop", spec.gripper))
    nodes.append(capture_browser)
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="openarm",
                description="Robot to run (see fm_bringup.registry). Remote mirror targets openarm.",
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="right_arm",
                description="Preset — single-arm for the rig's one tracked hand (openarm -> right_arm).",
            ),
            DeclareLaunchArgument(
                "sim_backend",
                default_value="mujoco",
                description="Sim backend hosting the controller_manager (macOS: mujoco or mock).",
            ),
            DeclareLaunchArgument(
                "use_foxglove",
                default_value="true",
                description="Start foxglove_bridge on ws://0.0.0.0:8765 (via sim.launch.py).",
            ),
            DeclareLaunchArgument(
                "teleop_delay",
                default_value="12.0",
                description="Seconds to hold pose_tracking back so the sim comes up first "
                "(pose_tracking_node FATAL-exits without a robot state within ~10 s).",
            ),
            DeclareLaunchArgument(
                "depth",
                default_value="on",
                description="on|off — drive the mirror from the rig's metric depth-Z "
                "(/vision/hand_point, per-hand for bimanual). off falls back to the mono "
                "/vision/<hand>/hand_pose.",
            ),
            DeclareLaunchArgument(
                "gripper",
                default_value="off",
                description="on|off — actuate the pinch grippers from hand open/close. "
                "Upgrades single right_arm to the pinch-gripper preset; bimanual carries "
                "grippers already, so this only toggles whether they move.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
