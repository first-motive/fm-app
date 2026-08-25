"""One-shot vision-mirror teleop session — the whole stack from a single launch.

This is what the fm_tui "Vision Teleop" action dispatches, so the operator does not juggle
three terminals (sim -> teleop -> engage) plus a separate recorder. It composes the existing
launch files rather than folding the sim into teleop.launch.py (whose docstring assumes the
sim is already up):

    sim.launch.py       robot_state_publisher (/robot_description) + foxglove_bridge
                        (ws://8765) + the sim backend + controllers
    teleop.launch.py    input:=mirror -> pose_tracking + hand_tracker + mirror_source,
                        started after `teleop_delay` so the sim's controllers + /joint_states
                        exist before pose_tracking_node's ~10 s robot-state wait
    mirror_datalogger   the /capture/record start/stop recorder (Foxglove REC/STOP buttons)
    arm_reset           /vision/reset -> disengage + drive the arm home (Foxglove RESET button)

Prerequisites: `camera_source` selects the camera when `camera_input=device` — default 0 (the
host's built-in webcam); a device index or stream URL also works. In Foxglove Studio, connect to
ws://localhost:8765 and import foxglove/mirror_teleop.json (Layouts -> Import from file).
"""

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fm_bringup import registry
from fm_bringup.sessions import include as _include

# Launch args forwarded verbatim into teleop.launch.py's mirror branch. Empty values keep the
# robot's vision.yaml / node defaults (teleop.launch.py only overrides when the arg is non-empty).
_TELEOP_FORWARD = ("camera_source", "rotate_deg", "publish_debug_image", "tracking_mode",
                   "hand_span_m", "capture_hands", "record_skeleton", "camera_input")


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    variant = LaunchConfiguration("variant").perform(context)
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove").perform(context)
    teleop_delay = float(LaunchConfiguration("teleop_delay").perform(context))

    spec = registry.get(robot)

    # gripper:=on upgrades the base right_arm to the pinch-gripper preset (URDF gripper + the
    # openarm_right_gripper_controller). An explicit non-right_arm variant (e.g. default_bimanual)
    # is left as-is since it already carries grippers. The effective variant flows to sim+teleop.
    want_gripper = (
        LaunchConfiguration("gripper").perform(context).strip().lower()
        in ("on", "true", "1", "yes")
    )
    if want_gripper and variant in ("", "right_arm"):
        variant = "right_arm_with_pinch_gripper"
    resolved_variant = variant or spec.default_variant
    # Per-arm mirror config when the variant drives both arms (default_bimanual); None for
    # single-arm. Drives the recorder joint prefixes, per-arm reset, and per-arm grippers below.
    bimanual_arms = spec.mirror_arms.get(resolved_variant)

    # 1) Sim owns /robot_description, robot_state_publisher, foxglove_bridge and the controllers.
    sim = _include(
        "sim.launch.py",
        {
            "robot": robot,
            "variant": variant,
            "sim_backend": sim_backend,
            "use_foxglove": use_foxglove,
        },
    )

    # 2) Mirror teleop, held back by teleop_delay so the sim's controllers + /joint_states are up
    # before pose_tracking_node's robot-state wait (it FATAL-exits if no state within ~10 s on a
    # cold sim). hand_tracker / mirror_source tolerate tf-not-ready, so only pose_tracking is
    # timing-sensitive. Tune teleop_delay if a cold MuJoCo start is slow.
    teleop_args = {
        "robot": robot,
        "variant": variant,
        "sim_backend": sim_backend,
        "input": "mirror",
    }
    for name in _TELEOP_FORWARD:
        teleop_args[name] = LaunchConfiguration(name).perform(context)
    teleop = TimerAction(period=teleop_delay, actions=[_include("teleop.launch.py", teleop_args)])

    # 3) Recorder — REC/STOP buttons publish /capture/record. Starts immediately; it intersects
    # its topic list at session start, so topics not-yet-up before teleop_delay are harmless.
    # The recorder reads the actual EE via tf; point it at the variant's EE frame (the gripper
    # preset renames the flange) so ee_*/err_* columns are populated, not empty.
    dl_ee = spec.ee_frames.get(resolved_variant)
    dl_args = ["--ee-frame", dl_ee] if dl_ee else []
    # Bimanual capture records BOTH arms' joints (the data engine reads the named `joints`
    # dict from the jsonl), so pass both arm prefixes.
    if bimanual_arms:
        dl_args += ["--arm-joint-prefix", "openarm_left_joint,openarm_right_joint"]
    datalogger = Node(
        package="fm_teleop_vision",
        executable="mirror_datalogger",
        name="mirror_datalogger",
        output="screen",
        arguments=dl_args,
        condition=IfCondition(LaunchConfiguration("record")),
    )

    # 4) Reset/home node(s) — RESET button publishes /vision/reset -> arm_reset disengages and
    # servos the EE to its home pose via /target_pose (Servo owns the controller, so a
    # joint_trajectory would be drowned by Servo's hold stream — see arm_reset.py).
    #
    # Single-arm: source command_frame from servo.yaml (the Servo planning frame mirror_source
    # stamps /target_pose in) so they never drift; the home EE pose is arm_reset's openarm
    # default (FK of the URDF spawn joints).
    #
    # Bimanual: one arm_reset per arm, each targeting its namespaced /<ns>/target_pose in the
    # arm's own base frame with the arm's home EE (from mirror_arms). Both share /vision/reset
    # and /vision/engage, so a single RESET disengages and drives BOTH arms home.
    if bimanual_arms:
        reset_nodes = [
            Node(
                package="fm_teleop_vision",
                executable="arm_reset",
                name="arm_reset_%s" % arm["ns"],
                output="screen",
                parameters=[{
                    "target_pose_topic": "/%s/target_pose" % arm["ns"],
                    "command_frame": arm["command_frame"],
                    "home_ee_position": list(arm["reset_home_ee"]["position"]),
                    "home_ee_orientation": list(arm["reset_home_ee"]["orientation"]),
                }],
            )
            for arm in bimanual_arms
        ]
    else:
        reset_params = {}
        try:
            with open(spec.servo_params_file()) as servo_file:
                servo_cfg = yaml.safe_load(servo_file)
            reset_params["command_frame"] = servo_cfg["moveit_servo"]["robot_link_command_frame"]
        except (OSError, KeyError, TypeError):
            pass  # keep arm_reset's own default command frame + home pose
        reset_nodes = [
            Node(
                package="fm_teleop_vision",
                executable="arm_reset",
                name="arm_reset",
                output="screen",
                parameters=[reset_params] if reset_params else [],
            )
        ]

    # 5) Capture browser — serves recorded sessions (index/detail) to the web GUI's
    # recordings viewer over the WS bridge. Always on (independent of record:=), so past
    # sessions are browsable even when not currently recording.
    capture_browser = Node(
        package="fm_teleop_vision",
        executable="capture_browser",
        name="capture_browser",
        output="screen",
    )

    # Startup seed — on the mujoco backend each arm's ros2_control system spawns its own sim
    # instance and the mirrored (left) arm's spawn initial_value does not reliably take, so it
    # sags. Command both arm controllers to the ready pose once (a few times, ~5 s in, before
    # teleop engages) so the arms hold symmetric. Sim only: NEVER on the real robot (it would
    # move a physical arm at bringup); the real motors hold their pose without seeding.
    seed_nodes = []
    if bimanual_arms and sim_backend != "real":
        seed_delay = max(4.0, teleop_delay - 4.0)
        for arm in bimanual_arms:
            rj = arm.get("ready_joints")
            if not rj:
                continue
            names = ",".join("openarm_%s_joint%d" % (arm["ns"], j) for j in range(1, 8))
            positions = ",".join(str(v) for v in rj)
            msg = ("{joint_names: [%s], points: [{positions: [%s], "
                   "time_from_start: {sec: 1}}]}" % (names, positions))
            seed_nodes.append(ExecuteProcess(
                cmd=["ros2", "topic", "pub", "-t", "4", "-r", "2",
                     "/openarm_%s_arm_controller/joint_trajectory" % arm["ns"],
                     "trajectory_msgs/msg/JointTrajectory", msg],
                output="screen"))
        if seed_nodes:
            seed_nodes = [TimerAction(period=seed_delay, actions=seed_nodes)]

    nodes = [sim, teleop, datalogger, *reset_nodes, *seed_nodes, capture_browser]

    # 6) Gripper adapter(s) — map each hand's mirror_source hand_preset (open|close) onto its
    # arm's gripper controller when gripper:=on. mirror_source already publishes the preset from
    # the hand's finger curl; this node moves the pinchers. Bimanual launches one per arm (each
    # hand drives its own pinchers, from mirror_arms); single-arm launches one from spec.gripper.
    # Note: gripper:=on does NOT change a default_bimanual variant (it already carries both
    # gripper controllers) — the variant upgrade above only fires for the single right_arm path.
    if want_gripper:
        if bimanual_arms:
            grip_specs = [
                (arm["ns"], arm["gripper"]) for arm in bimanual_arms if arm.get("gripper")
            ]
        elif spec.gripper:
            grip_specs = [("right", spec.gripper)]
        else:
            grip_specs = []
        for ns, g in grip_specs:
            nodes.append(
                Node(
                    package="fm_teleop_device",
                    executable="gripper_teleop",
                    name="gripper_teleop_%s" % ns if bimanual_arms else "gripper_teleop",
                    output="screen",
                    parameters=[{
                        "preset_topic": g["preset_topic"],
                        "command_topic": g["command_topic"],
                        "joints": list(g["joints"]),
                        "open_positions": list(g["open"]),
                        "close_positions": list(g["close"]),
                    }],
                )
            )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="openarm",
                description="Robot to run (see fm_bringup.registry). Mirror teleop targets openarm.",
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description="Preset; empty uses the registry default (openarm -> right_arm).",
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
                "camera_source",
                default_value="0",
                description="Camera device index (default 0 = the host's built-in webcam) "
                "or a stream URL. Used when camera_input=device.",
            ),
            DeclareLaunchArgument(
                "rotate_deg",
                default_value="",
                description="CLOCKWISE de-rotation (0|90|180|270) for a sideways stream. "
                "Empty keeps the vision.yaml default (0 for the Quest passthrough / webcam — "
                "already upright landscape; the iOS phone in portrait needs 90).",
            ),
            DeclareLaunchArgument(
                "publish_debug_image",
                default_value="true",
                description="Publish the /vision/image skeleton overlay (the Foxglove Image "
                "panel needs it). Default true for this dashboard session.",
            ),
            DeclareLaunchArgument(
                "tracking_mode",
                default_value="",
                description="hand (default) or full_body. Empty keeps the node default.",
            ),
            DeclareLaunchArgument(
                "hand_span_m",
                default_value="",
                description="Operator reference segment (m) for the metric scale. Empty keeps "
                "the vision.yaml / node default.",
            ),
            DeclareLaunchArgument(
                "capture_hands",
                default_value="right",
                description="right (control hand only) or both (track both hands for the "
                "dataset; the robot mirror still uses the right hand).",
            ),
            DeclareLaunchArgument(
                "record_skeleton",
                default_value="",
                description="Publish + record the hand-skeleton stream (true|false). Empty "
                "keeps the node default (on).",
            ),
            DeclareLaunchArgument(
                "camera_input",
                default_value="device",
                description="device (hand_tracker opens camera_source) or topic (start the "
                "fm_sensors head camera and track its ROS image stream — clean raw frames).",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="true",
                description="Start mirror_datalogger so the REC/STOP buttons record a session.",
            ),
            DeclareLaunchArgument(
                "teleop_delay",
                default_value="12.0",
                description="Seconds to hold the mirror teleop back so the sim comes up first "
                "(pose_tracking_node FATAL-exits without a robot state within ~10 s).",
            ),
            DeclareLaunchArgument(
                "gripper",
                default_value="off",
                description="on|off — run the right arm WITH the pinch gripper. Upgrades to the "
                "right_arm_with_pinch_gripper preset and launches gripper_teleop so the operator's "
                "hand open/close drives the pinchers.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
