"""Interactive teleop for the OpenArm — MoveIt Servo plus a selected input.

    input (foxglove | joy | spacenav)
        -> Servo (servo.launch.py)
        -> input source publishing TwistStamped/JointJog onto Servo's delta topics

Assumes the sim (or real) target is already up — run scripts/sim.sh in another
terminal first — so Servo has /joint_states and the arm's joint_trajectory_controller
to stream to.

    foxglove   no extra node: the browser panel publishes via foxglove_bridge (the
               primary, fleet-scalable input).
    joy        joy_node (Linux /dev/input, or a Mac host-side HID->Joy bridge)
               + joy_to_servo.
    spacenav   spacenav_node (USB, Linux only) + spacenav_to_servo.
    vision     vision_source: a camera tracks the operator's wrist and jogs the arm.
               Engage from the panel's "Vision (hold)" button. Needs the MediaPipe
               model (fm_teleop_vision/scripts/download_model.sh) and a camera — pass
               camera_source:=<index|url> (default 0, the host webcam).
    leader     leader_source: a physical leader arm's joints drive the follower's arm
               controller directly. This is the one input that runs NO Servo — the
               trajectory bypasses it — so the source owns the motion bounds. In sim,
               any publisher on leader_topic stands in for the arm.
    vr         vr_source: a tracked VR controller jogs the arm through Servo, drives
               the base from its thumbstick, and opens/closes the hand from its grip.
               Needs a VR bridge publishing PoseStamped + Joy.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fm_bringup import registry

_VALID_INPUTS = ("foxglove", "joy", "spacenav", "vision", "mirror", "leader", "vr")


def _command_frame(robot):
    """Read the robot's Servo command frame from the same servo.yaml Servo itself loads.

    A source's twist must be stamped in that frame, and it differs per robot
    (openarm_right_base_link, base_link, torso_link). Reading it here rather than
    restating it keeps the two from drifting.
    """
    servo_yaml = registry.get(robot).servo_params_file()
    try:
        with open(servo_yaml) as servo_file:
            servo_cfg = yaml.safe_load(servo_file)
        return servo_cfg["moveit_servo"]["robot_link_command_frame"]
    except (OSError, KeyError, TypeError) as exc:
        raise RuntimeError(
            f"Could not read robot_link_command_frame from {servo_yaml} for robot "
            f"'{robot}': {exc}. A jog twist must be stamped in that frame."
        ) from exc


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    teleop_input = LaunchConfiguration("input").perform(context)
    camera_source = LaunchConfiguration("camera_source").perform(context)
    model_path = LaunchConfiguration("model_path").perform(context)
    # Forwarded verbatim; servo.launch.py is the single point that resolves an
    # empty variant to the registry default, so robot/variant stay consistent.
    variant = LaunchConfiguration("variant").perform(context)

    if teleop_input not in _VALID_INPUTS:
        raise RuntimeError(
            f"Unknown input '{teleop_input}'. One of: {', '.join(_VALID_INPUTS)}."
        )

    # Bimanual mirror: the variant carries both arms and the registry declares a per-arm mirror
    # config, so both hands drive both arms — one namespaced pose_tracking + mirror_source per
    # arm. Any other case is single-arm and unchanged.
    spec_top = registry.get(robot)
    bimanual_arms = spec_top.mirror_arms.get(variant) if teleop_input == "mirror" else None

    def _pose_tracking(extra_args=None):
        args = {"robot": robot, "sim_backend": sim_backend, "variant": variant}
        args.update(extra_args or {})
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("fm_bringup"), "launch", "pose_tracking.launch.py")),
            launch_arguments=args.items(),
        )

    # mirror drives an ABSOLUTE EE pose target (1:1 hand mirroring), so it runs the
    # PoseTracking node — which embeds its own Servo — INSTEAD of servo_node. leader
    # streams a full trajectory to the arm controller and runs NEITHER. Every other input
    # jogs via servo_node. Exactly one writer owns the arm's JTC at a time.
    if teleop_input == "leader":
        nodes = []
    elif bimanual_arms:
        nodes = [_pose_tracking({
            "arm_ns": arm["ns"], "move_group": arm["move_group"],
            "command_frame": arm["command_frame"], "ee_frame": arm["ee_frame"],
            "command_out": arm["command_out"],
        }) for arm in bimanual_arms]
    elif teleop_input == "mirror":
        nodes = [_pose_tracking()]
    else:
        nodes = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("fm_bringup"), "launch", "servo.launch.py")),
                launch_arguments={
                    "robot": robot, "sim_backend": sim_backend, "variant": variant,
                }.items(),
            )
        ]

    if teleop_input == "leader":
        # The follower's controller identity comes from the registry (which reads the same
        # controllers.yaml the controller_manager loads), never from an operator-typed
        # topic — a stale joint order would command the wrong joints. Unlike the servo
        # path, which forwards an empty variant for servo.launch.py to resolve, the lookup
        # happens here: the registry is keyed by concrete variant.
        resolved_variant = variant or spec_top.default_variant
        arms = spec_top.arm_controllers(resolved_variant)
        if not arms:
            raise RuntimeError(
                f"Robot '{robot}' variant '{resolved_variant}' has no arm controller for "
                "the leader bypass."
            )
        leader_arm = LaunchConfiguration("leader_arm").perform(context)
        selected = next((arm for arm in arms if arm[0] == leader_arm), None) if leader_arm else arms[0]
        if selected is None:
            raise RuntimeError(
                f"leader_arm '{leader_arm}' is not an arm controller of '{robot}' "
                f"variant '{resolved_variant}'. One of: {', '.join(name for name, _ in arms)}."
            )
        controller, joints = selected
        nodes += [
            Node(
                package="fm_teleop_leader",
                executable="leader_source",
                output="screen",
                parameters=[{
                    "command_topic": f"/{controller}/joint_trajectory",
                    "joints": joints,
                    "leader_joint_states_topic": LaunchConfiguration(
                        "leader_topic"
                    ).perform(context),
                }],
            ),
        ]
    elif teleop_input == "vr":
        nodes += [
            Node(
                package="fm_teleop_vr",
                executable="vr_source",
                output="screen",
                parameters=[{"command_frame": _command_frame(robot)}],
            ),
        ]
    elif teleop_input == "joy":
        nodes += [
            Node(package="joy", executable="joy_node", output="screen"),
            Node(package="fm_teleop_device", executable="joy_to_servo", output="screen"),
        ]
    elif teleop_input == "spacenav":
        nodes += [
            Node(package="spacenav", executable="spacenav_node", output="screen"),
            Node(package="fm_teleop_device", executable="spacenav_to_servo", output="screen"),
        ]
    elif teleop_input == "vision":
        # Vision is the one input that needs launch-time config (camera, model, and the
        # jog-tuning knobs), so it gets a params dict where joy/spacenav rely on node
        # defaults. An empty model_path is omitted, not passed as "", so the node keeps
        # its own default. scale/deadzone/rate_hz are tuned per session: raise scale for
        # a stronger jog, lower rate_hz to cut CPU load when the sim is heavy.
        vision_params = {
            "camera_source": camera_source,
            "scale": float(LaunchConfiguration("scale").perform(context)),
            "deadzone": float(LaunchConfiguration("deadzone").perform(context)),
            "rate_hz": float(LaunchConfiguration("rate_hz").perform(context)),
        }
        if model_path:
            vision_params["model_path"] = model_path
        vision_params["command_frame"] = _command_frame(robot)
        nodes += [
            Node(
                package="fm_teleop_vision",
                executable="vision_source",
                output="screen",
                parameters=[vision_params],
            ),
        ]
    elif teleop_input == "mirror":
        # 1:1 hand mirroring: hand_tracker (MediaPipe Hands) + mirror_source (absolute pose
        # target -> the PoseTracking node included above, not servo_node). Params load from
        # the robot's vision.yaml (the persisted home for tuned values); launch args are
        # appended AFTER so an explicitly-set one wins, while an empty one keeps the yaml.
        spec = registry.get(robot)
        vision_yaml = spec.vision_params_file()
        base_params = [vision_yaml] if os.path.exists(vision_yaml) else []

        tracker_overrides = {"camera_source": camera_source}
        rotate_deg = LaunchConfiguration("rotate_deg").perform(context)
        if rotate_deg:
            tracker_overrides["rotate_deg"] = int(rotate_deg)
        debug_image = LaunchConfiguration("publish_debug_image").perform(context)
        if debug_image:
            tracker_overrides["publish_debug_image"] = debug_image.lower() in ("true", "1", "yes")
        tracking_mode = LaunchConfiguration("tracking_mode").perform(context)
        if tracking_mode:
            tracker_overrides["tracking_mode"] = tracking_mode
        # hand_tracker resolves its model (hand vs pose) from the package share dir by
        # tracking_mode, so model_path is NOT forced here (unlike the vision wrist path).

        # Dataset capture knobs: track BOTH hands (control still uses one), publish+record
        # the hand-skeleton stream, and optionally take frames from a ROS camera topic (the
        # fm_sensors head-cam) so the recorded frames are exactly the tracked frames.
        camera_nodes = []
        if bimanual_arms or LaunchConfiguration("capture_hands").perform(context) == "both":
            tracker_overrides["num_hands"] = 2   # bimanual control needs both hands tracked
        record_skeleton = LaunchConfiguration("record_skeleton").perform(context)
        if record_skeleton:
            tracker_overrides["publish_skeleton"] = record_skeleton.lower() in ("true", "1", "yes")
        if LaunchConfiguration("camera_input").perform(context) == "topic":
            tracker_overrides["input_mode"] = "topic"
            camera_nodes.append(
                Node(
                    package="fm_sensors",
                    executable="camera_node",
                    name="head_camera",
                    output="screen",
                    parameters=[{
                        "camera_source": camera_source,
                        "frame_id": "head_cam",
                        "image_topic": "head_cam/image_raw",
                        "info_topic": "head_cam/camera_info",
                    }],
                )
            )

        # mirror_source: stamp the target in the robot's Servo command frame (read from the
        # same servo.yaml Servo loads, so they never drift), plus the metric-scale knob.
        source_overrides = {}
        servo_yaml = spec.servo_params_file()
        try:
            with open(servo_yaml) as servo_file:
                servo_cfg = yaml.safe_load(servo_file)
            source_overrides["command_frame"] = servo_cfg["moveit_servo"][
                "robot_link_command_frame"
            ]
        except (OSError, KeyError, TypeError) as exc:
            raise RuntimeError(
                f"Could not read robot_link_command_frame from {servo_yaml} for robot "
                f"'{robot}': {exc}. The mirror target must be stamped in that frame."
            ) from exc
        # Anchor the mirror at the variant's EE frame (matches servo's ee_frame_name). The
        # pinch-gripper preset renames the flange link7 -> ee_base_link; without this the
        # tf(command_frame -> ee_frame) lookup fails and mirror_source never latches / commands.
        ee_frame = spec.ee_frames.get(variant or spec.default_variant)
        if ee_frame:
            source_overrides["ee_frame"] = ee_frame
        hand_span = LaunchConfiguration("hand_span_m").perform(context)
        if hand_span:
            source_overrides["hand_span_m"] = float(hand_span)
        elif tracking_mode == "full_body":
            # full_body sizes depth by the FOREARM (elbow->wrist), not the hand span.
            source_overrides["hand_span_m"] = 0.26

        hand_tracker_node = Node(
            package="fm_teleop_vision",
            executable="hand_tracker",
            name="hand_tracker",
            output="screen",
            parameters=base_params + [tracker_overrides],
        )
        if bimanual_arms:
            # One tracker (both hands) feeds one mirror_source per arm, each reading its hand's
            # pose stream and driving its arm's namespaced pose_tracking + gripper.
            mirror_nodes = []
            for arm in bimanual_arms:
                per_arm = dict(source_overrides)
                per_arm.update({
                    "command_frame": arm["command_frame"],
                    "ee_frame": arm["ee_frame"],
                    "target_pose_topic": "/%s/target_pose" % arm["ns"],
                    "hand_pose_topic": "vision/%s/hand_pose" % arm["hand"],
                    "grip_topic": "vision/%s/grip" % arm["hand"],
                    "tracking_topic": "vision/%s/tracking_active" % arm["hand"],
                    "gripper_preset_topic": arm["gripper_preset"],
                })
                # Per-arm overrides (left's mirrored workspace box + axis-map; both arms'
                # mirror_gain); appended after base_params so they override the vision.yaml
                # defaults tuned for the single right arm.
                for key in ("workspace_min", "workspace_max", "axis_map_linear", "mirror_gain"):
                    if key in arm:
                        per_arm[key] = arm[key]
                mirror_nodes.append(Node(
                    package="fm_teleop_vision", executable="mirror_source",
                    name="mirror_source_%s" % arm["ns"], output="screen",
                    parameters=base_params + [per_arm]))
            nodes += camera_nodes + [hand_tracker_node] + mirror_nodes
        else:
            nodes += camera_nodes + [
                hand_tracker_node,
                Node(
                    package="fm_teleop_vision",
                    executable="mirror_source",
                    name="mirror_source",
                    output="screen",
                    parameters=base_params + [source_overrides],
                ),
            ]
    # foxglove: the browser panel is the publisher; no ROS-side input node.

    # Robot-specific teleop adapters (e.g. the G1-D hand teleop, which maps the panel's
    # hand presets/sliders onto the hand controllers). Registry-driven, so this file holds
    # no robot-specific data.
    for package, executable in registry.get(robot).teleop_nodes:
        nodes.append(Node(package=package, executable=executable, output="screen"))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="openarm",
                description="Robot to teleop (see fm_bringup.registry).",
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description="Preset; must match the running sim. Empty uses the "
                "registry default. Servo's SRDF + description follow it.",
            ),
            DeclareLaunchArgument(
                "sim_backend",
                default_value="mujoco",
                description="Backend the running target uses (parses the description).",
            ),
            DeclareLaunchArgument(
                "input",
                default_value="foxglove",
                description="foxglove | joy | spacenav | vision (wrist jog) | mirror "
                "(1:1 hand mirroring via PoseTracking) | leader (leader arm, bypasses "
                "Servo) | vr (tracked controller).",
            ),
            DeclareLaunchArgument(
                "leader_topic",
                default_value="/leader/joint_states",
                description="leader input only: the leader arm's JointState topic. In sim "
                "any publisher on it stands in for the arm.",
            ),
            DeclareLaunchArgument(
                "leader_arm",
                default_value="",
                description="leader input only: which arm controller the leader drives on a "
                "multi-arm variant. Empty takes the first.",
            ),
            DeclareLaunchArgument(
                "camera_source",
                default_value="0",
                description="vision input only: webcam index or stream URL "
                "(e.g. http://<phone-ip>:8080/video).",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="/ws/fm_teleop/fm_teleop_vision/models/"
                "pose_landmarker_heavy.task",
                description="vision input only: MediaPipe pose model path. Default is "
                "the bind-mounted package models/ dir (download_model.sh writes there). "
                "Empty falls back to the node's own default.",
            ),
            DeclareLaunchArgument(
                "scale",
                default_value="4.0",
                description="vision input only: wrist displacement (m) -> jog velocity. "
                "Raise for a stronger, more visible jog.",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.03",
                description="vision input only: per-axis displacement (m) below which the "
                "jog is zero.",
            ),
            DeclareLaunchArgument(
                "rate_hz",
                default_value="30.0",
                description="vision input only: capture + command rate. Lower (e.g. 15) "
                "to cut CPU load when the sim is heavy.",
            ),
            DeclareLaunchArgument(
                "rotate_deg",
                default_value="",
                description="mirror input only: CLOCKWISE de-rotation (0|90|180|270) for a "
                "sideways phone stream. Empty keeps the vision.yaml / node default.",
            ),
            DeclareLaunchArgument(
                "publish_debug_image",
                default_value="",
                description="mirror input only: publish the vision/image skeleton overlay "
                "(true|false) to validate tracking. Empty keeps the vision.yaml default.",
            ),
            DeclareLaunchArgument(
                "tracking_mode",
                default_value="",
                description="mirror input only: hand (Hand Landmarker + grip) or full_body "
                "(Pose Landmarker; body wrist drives the arm). Empty keeps the default (hand).",
            ),
            DeclareLaunchArgument(
                "hand_span_m",
                default_value="",
                description="mirror input only: operator's reference segment in metres for "
                "the metric scale (wrist->knuckle ~0.09 hand, forearm ~0.26 full_body). "
                "Empty keeps the vision.yaml / node default.",
            ),
            DeclareLaunchArgument(
                "capture_hands",
                default_value="right",
                description="mirror input only: right (control hand only) or both (track "
                "both hands for the dataset; control still uses the right hand).",
            ),
            DeclareLaunchArgument(
                "record_skeleton",
                default_value="",
                description="mirror input only: publish + record the hand-skeleton stream "
                "(true|false). Empty keeps the node default (on).",
            ),
            DeclareLaunchArgument(
                "camera_input",
                default_value="device",
                description="mirror input only: device (hand_tracker opens camera_source) or "
                "topic (start the fm_sensors head camera and track its ROS image stream).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
