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

    spec = registry.get(robot)
    resolved_variant = variant or spec.default_variant

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

    # 2) PoseTracking (embeds Servo; owns the arm's JTC on the mirror path), held back by
    # teleop_delay so the sim's controllers + /joint_states are up before the node's ~10 s
    # robot-state wait (a cold MuJoCo start otherwise FATAL-exits it). mirror_source tolerates
    # tf-not-ready and only latches on engage, so only pose_tracking is timing-sensitive.
    pose_tracking = TimerAction(
        period=teleop_delay,
        actions=[
            _include(
                "pose_tracking.launch.py",
                {"robot": robot, "sim_backend": sim_backend, "variant": variant},
            )
        ],
    )

    # 3) Bare mirror_source — NO local camera/tracker. It subscribes the REMOTE rig's
    # /vision/hand_pose (+ grip/tracking/engage) over DDS and publishes the absolute EE target on
    # /target_pose. Params load from the robot's vision.yaml; the target is stamped in the Servo
    # command frame (read from the same servo.yaml Servo loads, so they never drift) and anchored
    # at the variant's EE frame (the tf(command_frame -> ee_frame) lookup gates engage/commands).
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
    source_overrides = {"command_frame": command_frame}
    ee_frame = spec.ee_frames.get(resolved_variant)
    if ee_frame:
        source_overrides["ee_frame"] = ee_frame
    # Depth-Z: drive the mirror from the rig's REAL metric depth (/vision/hand_point) instead of
    # the mono apparent-size z. Needs mirror_source's metric input mode (fm-teleop). depth:=off
    # falls back to the mono /vision/hand_pose (no mirror_source change needed). The metric depth
    # axis is enabled here (axis_gain[2]=1); its sign is camera-optical-frame specific — tune on
    # the rig and persist in vision.yaml.
    if LaunchConfiguration("depth").perform(context).strip().lower() in ("on", "true", "1", "yes"):
        source_overrides["input_mode"] = "metric"
        source_overrides["hand_point_topic"] = "/vision/hand_point"
        source_overrides["axis_gain"] = [1.0, 1.0, 1.0]
    mirror_source = Node(
        package="fm_teleop_vision",
        executable="mirror_source",
        name="mirror_source",
        output="screen",
        parameters=base_params + [source_overrides],
    )

    # 4) Reset/home — the RESET button publishes /vision/reset; arm_reset disengages and servos the
    # EE home via /target_pose. Stamp its home in the same command frame so it never drifts.
    arm_reset = Node(
        package="fm_teleop_vision",
        executable="arm_reset",
        name="arm_reset",
        output="screen",
        parameters=[{"command_frame": command_frame}],
    )

    # 5) Capture browser — serve recorded sessions to the web GUI (recording itself is rig-side).
    capture_browser = Node(
        package="fm_teleop_vision",
        executable="capture_browser",
        name="capture_browser",
        output="screen",
    )

    return [sim, pose_tracking, mirror_source, arm_reset, capture_browser]


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
                "(/vision/hand_point). off falls back to the mono /vision/hand_pose.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
