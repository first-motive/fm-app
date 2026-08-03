"""Robot registry for the fm_bringup control launch layer.

One entry per robot owns everything the control launch files vary on, so adding a
robot is a single :class:`RobotSpec` here instead of edits scattered across
``sim.launch.py`` / ``teleop.launch.py`` / ``servo.launch.py``. Mirrors
``fm_description``'s ``view_robot.launch.py``, which did the same for description
views.

Each entry carries:

    description   backend-selectable xacro + per-robot visual-mesh rewrite
    controllers   per-variant active/inactive sets, which backends need a
                  standalone controller_manager, and any cmd_vel remaps
    foxglove      foxglove_bridge params (mesh allowlist, buffer limits)
    servo         MoveIt Servo context: SRDF locator, MoveIt config package,
                  servo.yaml (plus extra servo_nodes for multi-arm robots)

The launch files read a spec via :func:`get` and call its helpers; they hold no
robot-specific data themselves.
"""

import os
import re
from dataclasses import dataclass, field
from typing import Optional

import xacro
from ament_index_python.packages import get_package_share_directory

from fm_sim_models.models import mjcf_path

# --- OpenArm specifics -------------------------------------------------------

# Visual meshes ship as z-up .stl under fm_description (converted at build); the
# upstream xacro points visuals at openarm_description .dae. Rewrite so Foxglove
# (fed by robot_state_publisher) renders. Collisions stay on openarm_description.
_OPENARM_MESH_RE = re.compile(r"package://openarm_description/([^\"']+?)\.dae")
_OPENARM_MESH_SUB = r"package://fm_description/openarm_meshes/\1.stl"

# The vendored SO101 URDF references meshes by relative path (assets/...). Rewrite to
# package:// so robot_state_publisher serves them to Foxglove, matching where
# fm_description installs them (share/fm_description/so101_description/assets/).
_SO101_MESH_RE = re.compile(r'filename="assets/')
_SO101_MESH_SUB = r'filename="package://fm_description/so101_description/assets/'

# Same for the vendored G1-D URDF (relative meshes/... paths).
_G1_MESH_RE = re.compile(r'filename="meshes/')
_G1_MESH_SUB = r'filename="package://fm_description/g1_d_description/meshes/'

# The vendored Axol URDF references meshes as package://assembly/... (its Onshape
# export package name). Rewrite the "assembly" package to fm_description/axol_description
# so robot_state_publisher serves them to Foxglove — matching where fm_description
# installs them and the _build_axol rewrite in view_robot.launch.py.
_AXOL_MESH_RE = re.compile(r'filename="package://assembly/')
_AXOL_MESH_SUB = r'filename="package://fm_description/axol_description/'

# foxglove_bridge params shared across robots: the default asset_uri_allowlist ([\w-]
# only) rejects package:// paths through a dotted directory (e.g. the OpenArm's
# openarm_v2.0), so nothing renders; [-\w.] admits the dot. send_buffer_limit is
# raised above the 10 MB default for large body meshes.
DEFAULT_FOXGLOVE_PARAMS = {
    "port": 8765,
    "address": "0.0.0.0",
    "send_buffer_limit": 134217728,
    "asset_uri_allowlist": [
        r"^package://(?:[-\w.]+/)*[-\w.]+"
        r"\.(?:dae|stl|obj|glb|gltf|mtl|png|jpe?g|tiff?)$"
    ],
}


@dataclass(frozen=True)
class RobotSpec:
    """Everything the control launch files need to drive one robot.

    Helpers resolve share paths lazily (at launch time, inside the container)
    rather than at import, so the spec is a plain data record.
    """

    key: str
    label: str
    default_variant: str

    # description
    control_xacro: str  # filename under fm_control/urdf
    preset_arg: Optional[str]  # xacro arg the variant maps to, or None for single-config robots
    mesh_rewrite: Optional[tuple]  # (compiled_regex, repl) applied to the URDF, or None

    # controllers
    config_dir: str  # subdir under fm_bringup/config holding this robot's configs
    controllers: dict  # variant -> {"active": [...], "inactive": [...]}
    standalone_cm_backends: frozenset  # backends needing a standalone controller_manager

    # foxglove
    foxglove_params: dict

    # servo
    moveit_pkg: str  # vendored MoveIt config package (kinematics, joint limits)
    moveit_cfg: str  # config subdir within that package
    servo_config: str  # servo.yaml filename under config/<config_dir>
    bringup_srdf: dict  # variant -> SRDF filename under config/<config_dir>
    moveit_srdf: str  # fallback SRDF filename in the MoveIt config package

    # Extra MoveIt Servo instances beyond the primary, as ((node_name, servo.yaml), ...).
    # Multi-arm robots (the G1-D) run one servo_node per arm group, each with its own
    # servo.yaml + delta command topics under /<node_name>/. Empty for single-arm robots.
    extra_servo_configs: tuple = ()

    # Topic remaps applied to the standalone controller_manager, as ((from, to), ...).
    # Used to put a controller's fixed topic on a canonical name (e.g. diff_drive's
    # cmd_vel_unstamped -> /cmd_vel). Empty for robots that need no remap.
    cmd_remaps: tuple = ()

    # Extra teleop adapter nodes launched alongside Servo for the panel, as
    # ((package, executable), ...). The G1-D runs g1_hand_teleop to map the panel's hand
    # presets/sliders onto the hand controllers. Empty for arm-only robots.
    teleop_nodes: tuple = ()

    # When the controllers drive only a SUBSET of the model's joints (the G1-D arm is
    # 7 of 34), the unactuated joints never reach /joint_states, so MoveIt's planning
    # scene monitor never completes ("complete state not known") and Servo will not jog.
    # True adds a joint_state_publisher that fills those joints at their default while
    # taking the controlled joints from the broadcaster via source_list.
    full_state_jsp: bool = False

    # Per-variant "home"/ready joint positions, matching the URDF spawn pose in the control
    # xacro (fm_control). Consumed by the vision RESET/HOME path: vision_session.launch.py
    # passes home_positions[variant] to fm_teleop_vision/arm_reset, which drives the arm here
    # on reset. Kept as data (not re-parsed from the xacro) but must stay in sync with it.
    # variant -> [pos per controlled arm joint, in controllers.yaml joint order]. Empty for
    # robots without a defined home.
    home_positions: dict = field(default_factory=dict)

    # Optional pinch-gripper adapter spec. When the vision session runs with gripper:=on,
    # vision_session.launch.py launches fm_teleop_device/gripper_teleop with these params to
    # map mirror_source's hand_preset (open|close) onto the gripper controller. None for robots
    # with no gripper in the vision path. Keys: preset_topic, command_topic, joints, open, close.
    gripper: Optional[dict] = None

    # Per-variant end-effector (servo tip / mirror anchor) frame. The pinch-gripper preset
    # REPLACES the bare flange link7 with the gripper's ee_base_link, so servo's ee_frame_name
    # and mirror_source's ee_frame must follow the variant or the arm can't be servoed / latched.
    # Empty -> keep servo.yaml / node defaults. variant -> frame id.
    ee_frames: dict = field(default_factory=dict)

    # Per-arm config for the BIMANUAL vision mirror (both hands -> both arms). Empty for
    # single-arm robots. Each entry drives one hand's pose stream to one arm through its own
    # mirror_source + pose_tracking_node instance, namespaced by `ns` so their /target_pose,
    # ~/status etc. never collide. Keyed to a variant that carries both arms' controllers.
    #   ns             topic namespace for the arm's pose_tracking (-> /<ns>/target_pose)
    #   hand           which tracked hand drives it ("left" | "right")
    #   command_frame  Servo planning frame = tf frame the EE is read in
    #   ee_frame       the arm tip link (mirror anchor + Servo ee_frame_name)
    #   move_group     SRDF planning group for this arm
    #   command_out    the arm's joint_trajectory_controller topic
    #   gripper_preset the arm's gripper preset topic (hand open/close)
    mirror_arms: dict = field(default_factory=dict)  # variant -> tuple(dict per arm)

    # --- path helpers --------------------------------------------------------

    def _config(self, *parts):
        return os.path.join(
            get_package_share_directory("fm_bringup"), "config", self.config_dir, *parts
        )

    def controllers_file(self, variant):
        return self._config(f"{variant}.controllers.yaml")

    def servo_params_file(self):
        return self._config(self.servo_config)

    def pose_tracking_params_file(self):
        """Abs path to pose_tracking.yaml (PoseTracking PID + command_in_type deltas over
        servo.yaml, for the vision 1:1 mirror path). May not exist for every robot."""
        return self._config("pose_tracking.yaml")

    def vision_params_file(self):
        """Abs path to vision.yaml (hand_tracker + mirror_source params for input:=mirror —
        rotate_deg, mirror_gain, workspace box). May not exist for every robot; callers
        guard with os.path.exists and fall back to the node defaults."""
        return self._config("vision.yaml")

    def servo_nodes(self):
        """Return ``[(node_name, servo.yaml abs path), ...]`` — primary plus extras.

        Multi-arm robots run one servo_node per arm group, each with its own
        servo.yaml and delta command topics under ``/<node_name>/``. Single-arm
        robots return just the primary ``servo_node``.
        """
        nodes = [("servo_node", self.servo_params_file())]
        nodes += [(name, self._config(cfg)) for name, cfg in self.extra_servo_configs]
        return nodes

    def moveit_file(self, name):
        return os.path.join(
            get_package_share_directory(self.moveit_pkg), self.moveit_cfg, name
        )

    # --- builders ------------------------------------------------------------

    def build_description(self, variant, sim_backend, controllers_file=None):
        """Process the backend-selectable xacro into a description string.

        ``controllers_file`` is baked in only for the gazebo backend, whose
        controller_manager lives inside the description plugin.
        """
        xacro_path = os.path.join(
            get_package_share_directory("fm_control"), "urdf", self.control_xacro
        )
        mappings = {"sim_backend": sim_backend}
        # Single-config robots (preset_arg=None) take no preset; the variant is nominal.
        if self.preset_arg:
            mappings[self.preset_arg] = variant
        # The mujoco backend reads its MJCF from the description's mujoco_model param;
        # the path is owned by fm_sim_models, not hardcoded in the xacro.
        if sim_backend == "mujoco":
            mappings["mujoco_model"] = mjcf_path(self.key)
        if sim_backend == "gazebo" and controllers_file:
            mappings["gazebo_controllers_file"] = controllers_file
        xml = xacro.process_file(xacro_path, mappings=mappings).toxml()
        if self.mesh_rewrite:
            pattern, repl = self.mesh_rewrite
            xml = pattern.sub(repl, xml)
        return xml

    def semantic(self, variant):
        """Read the SRDF matching the variant.

        Variants listed in ``bringup_srdf`` use an in-repo SRDF (e.g. the
        single-arm right_arm); everything else falls back to the vendored MoveIt
        config's SRDF.
        """
        if variant in self.bringup_srdf:
            path = self._config(self.bringup_srdf[variant])
        else:
            path = self.moveit_file(self.moveit_srdf)
        with open(path, "r") as handle:
            return handle.read()


_ROBOTS = {
    "openarm": RobotSpec(
        key="openarm",
        label="Enactic OpenArm",
        default_variant="right_arm",
        control_xacro="openarm.sim.urdf.xacro",
        preset_arg="robot_preset",
        mesh_rewrite=(_OPENARM_MESH_RE, _OPENARM_MESH_SUB),
        config_dir="openarm",
        controllers={
            "right_arm": {
                "active": ["openarm_right_arm_controller"],
                "inactive": ["openarm_right_forward_position_controller"],
            },
            "right_arm_with_pinch_gripper": {
                "active": [
                    "openarm_right_arm_controller",
                    "openarm_right_gripper_controller",
                ],
                "inactive": [],
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
        },
        standalone_cm_backends=frozenset({"mock", "real"}),
        foxglove_params=DEFAULT_FOXGLOVE_PARAMS,
        moveit_pkg="openarm_bimanual_moveit_config",
        moveit_cfg=os.path.join("config", "openarm_v2.0"),
        servo_config="servo.yaml",
        # The gripper variant reuses the single-arm SRDF — the servo planning group (the 7 arm
        # joints) is unchanged; the extra finger joints live outside it.
        bringup_srdf={
            "right_arm": "right_arm.srdf",
            "right_arm_with_pinch_gripper": "right_arm.srdf",
        },
        moveit_srdf="openarm_bimanual.srdf",
        # Bent "ready" spawn pose from openarm.ros2_control.xacro (joint2/4/6 = 0.5/1.2/0.3,
        # rest 0) — off the straight-arm singularity. arm_reset drives here on RESET.
        home_positions={
            "right_arm": [0.0, 0.5, 0.0, 1.2, 0.0, 0.3, 0.0],
            "right_arm_with_pinch_gripper": [0.0, 0.5, 0.0, 1.2, 0.0, 0.3, 0.0],
            # "Hands forward, ~20cm apart" bimanual ready pose; left (first, per preset order)
            # is the right mirrored about the sagittal plane. Mirrors the URDF spawn
            # (openarm.sim.urdf.xacro ready_bimanual). Dead field — nothing reads
            # home_positions; kept in sync with the URDF for reference.
            "default_bimanual": [-0.6816, -0.0265, 0.3161, 1.1622, 0.0, -0.3, 0.0]
            + [0.6816, 0.0265, -0.3161, 1.1622, 0.0, 0.3, 0.0],
        },
        # Pinch gripper: hand open/close -> finger_joint1 position. Right finger range is
        # [-0.7854, 0]; live testing showed the physical open/close is inverted from the joint
        # sign, so open = -0.7854 (fingers apart), close = 0.0 (fingers together). joint2 mimics.
        gripper={
            "preset_topic": "/gripper_teleop/right/preset",
            "command_topic": "/openarm_right_gripper_controller/joint_trajectory",
            "joints": ["openarm_right_finger_joint1"],
            "open": [-0.7854],
            "close": [0.0],
        },
        # link7 is the bare flange; the pinch-gripper preset replaces it with ee_base_link
        # (measured at the identical home pose, so arm_reset's home EE is unchanged).
        ee_frames={
            "right_arm": "openarm_right_link7",
            "right_arm_with_pinch_gripper": "openarm_right_ee_base_link",
            # Bimanual arms carry the pinch gripper, so the tip is ee_base_link (link7 is the
            # bare flange and is NOT emitted by the default_bimanual preset — the model ends at
            # link6 + ee_base_link, same as right_arm_with_pinch_gripper).
            "default_bimanual": "openarm_right_ee_base_link",   # right is the primary servo tip
        },
        # Bimanual mirror: right hand -> right arm, left hand -> left arm. Each arm gets its
        # own namespaced pose_tracking + a mirror_source. left_arm is the vendored SRDF's
        # second planning group (mirror of right_arm). Each entry also carries the arm's
        # gripper spec (per-hand pinchers), reset home EE pose, and — for the left, whose
        # base frame is the right mirrored about the sagittal (XZ) plane — its own workspace
        # box + axis-map so it does not inherit the right-tuned values from vision.yaml.
        mirror_arms={
            "default_bimanual": (
                {"ns": "right", "hand": "right",
                 "command_frame": "openarm_right_base_link", "ee_frame": "openarm_right_ee_base_link",
                 "move_group": "right_arm",
                 "command_out": "/openarm_right_arm_controller/joint_trajectory",
                 "gripper_preset": "/gripper_teleop/right/preset",
                 # Startup seed = the URDF ready pose. On mujoco each arm's ros2_control system
                 # spawns its own sim instance and the mirrored (left) arm's initial_value does
                 # not always take, so it sags; seeding both arm controllers once at bringup
                 # holds them at the symmetric ready pose (see vision_session seed_ready).
                 "ready_joints": [0.6816, 0.0265, -0.3161, 1.1622, 0.0, 0.3, 0.0],
                 # 1:1 (not vision.yaml's 0.75) so hands-together maps to grippers-together.
                 "mirror_gain": 1.0,
                 "gripper": {
                     "preset_topic": "/gripper_teleop/right/preset",
                     "command_topic": "/openarm_right_gripper_controller/joint_trajectory",
                     "joints": ["openarm_right_finger_joint1"],
                     "open": [-0.7854], "close": [0.0],
                 },
                 # EE of the "hands forward, ~20cm apart" bimanual ready pose (base frame),
                 # matching the URDF spawn (openarm.sim.urdf.xacro ready_bimanual).
                 "reset_home_ee": {
                     "position": [0.3399, -0.069, -0.12],
                     "orientation": [0.0012, -0.6872, 0.1579, 0.709],
                 }},
                {"ns": "left", "hand": "left",
                 "command_frame": "openarm_left_base_link", "ee_frame": "openarm_left_ee_base_link",
                 "move_group": "left_arm",
                 "command_out": "/openarm_left_arm_controller/joint_trajectory",
                 "gripper_preset": "/gripper_teleop/left/preset",
                 "ready_joints": [-0.6816, -0.0265, 0.3161, 1.1622, 0.0, -0.3, 0.0],
                 "mirror_gain": 1.0,
                 # Right Y range [-0.45, 0.30] mirrors to [-0.30, 0.45]; X/Z unchanged.
                 # Tune the face magnitudes / axis-map signs live if the left clamps or
                 # feels axis-inverted (mapping is otherwise identical to the right).
                 "workspace_min": [-0.10, -0.30, -0.55],
                 "workspace_max": [0.55, 0.45, 0.25],
                 "axis_map_linear": ["z", "x", "-y"],
                 # Left finger axis is X, the reflect is on Y -> range unchanged from right.
                 "gripper": {
                     "preset_topic": "/gripper_teleop/left/preset",
                     "command_topic": "/openarm_left_gripper_controller/joint_trajectory",
                     "joints": ["openarm_left_finger_joint1"],
                     "open": [-0.7854], "close": [0.0],
                 },
                 # Sagittal mirror of the right reset home (Y negated; quat x,z negated).
                 "reset_home_ee": {
                     "position": [0.3399, 0.069, -0.12],
                     "orientation": [-0.0012, -0.6872, -0.1579, 0.709],
                 }},
            ),
        },
    ),
    "so101": RobotSpec(
        key="so101",
        label="LeRobot SO101",
        default_variant="so101",
        control_xacro="so101.sim.urdf.xacro",
        preset_arg=None,  # single fixed configuration; the variant is nominal
        mesh_rewrite=(_SO101_MESH_RE, _SO101_MESH_SUB),
        config_dir="so101",
        controllers={
            "so101": {
                "active": ["so101_arm_controller", "so101_gripper_controller"],
                "inactive": [],
            },
        },
        # real is the genuine feetech ros2_control plugin, so it needs a standalone
        # controller_manager just like mock (unlike the G1, whose real path is a bridge).
        standalone_cm_backends=frozenset({"mock", "real"}),
        foxglove_params=DEFAULT_FOXGLOVE_PARAMS,
        # SO101 MoveIt config is authored in-repo (Humble, bare joint names), so the
        # MoveIt files live under fm_bringup/config/so101 rather than a vendored package.
        moveit_pkg="fm_bringup",
        moveit_cfg=os.path.join("config", "so101"),
        servo_config="servo.yaml",
        bringup_srdf={"so101": "so101.srdf"},
        moveit_srdf="so101.srdf",
    ),
    "g1_d": RobotSpec(
        key="g1_d",
        label="Unitree",
        default_variant="g1_d",
        control_xacro="g1.sim.urdf.xacro",
        preset_arg=None,  # single fixed configuration; the variant is nominal
        mesh_rewrite=(_G1_MESH_RE, _G1_MESH_SUB),
        config_dir="g1_d",
        controllers={
            "g1_d": {
                "active": [
                    "g1_right_arm_controller",
                    "g1_left_arm_controller",
                    "g1_base_controller",
                    "g1_right_hand_controller",
                    "g1_left_hand_controller",
                ],
                "inactive": [],
            },
        },
        # diff_drive_controller subscribes ~/cmd_vel_unstamped; remap it to the canonical
        # /cmd_vel so the panel + g1_base_teleop share one base topic across sim and real.
        cmd_remaps=(("/g1_base_controller/cmd_vel_unstamped", "/cmd_vel"),),
        # Panel hand presets/sliders -> hand controllers via the hand teleop adapter.
        teleop_nodes=(("fm_teleop_device", "g1_hand_teleop"),),
        # Only mock needs a standalone controller_manager; mujoco/gazebo/isaac host
        # their own. real is NOT here — the G1 has no ros2_control hardware interface,
        # so the real arm is driven by the Servo->arm_sdk bridge (g1_arm_sdk_bridge),
        # not a controller_manager. sim.launch therefore serves the sim backends only.
        standalone_cm_backends=frozenset({"mock"}),
        foxglove_params=DEFAULT_FOXGLOVE_PARAMS,
        # G1-D MoveIt config is authored in-repo (Humble, right-arm subset).
        moveit_pkg="fm_bringup",
        moveit_cfg=os.path.join("config", "g1_d"),
        servo_config="servo.yaml",
        bringup_srdf={"g1_d": "g1_d.srdf"},
        moveit_srdf="g1_d.srdf",
        # Second servo_node for the left arm: its own servo.yaml + delta topics under
        # /servo_node_left/ (the primary servo_node drives the right arm).
        extra_servo_configs=(("servo_node_left", "servo_left.yaml"),),
        # Servo drives 14 of the G1-D's 34 joints; fill the rest so the planning scene
        # completes (see full_state_jsp above).
        full_state_jsp=True,
    ),
    "axol": RobotSpec(
        key="axol",
        label="Almond Bot Axol",
        default_variant="axol",  # single fixed bimanual configuration; nominal variant
        control_xacro="axol.sim.urdf.xacro",
        preset_arg=None,
        mesh_rewrite=(_AXOL_MESH_RE, _AXOL_MESH_SUB),
        config_dir="axol",
        controllers={
            "axol": {
                "active": [
                    "axol_right_arm_controller",
                    "axol_left_arm_controller",
                ],
                "inactive": [],
            },
        },
        # Only mock needs a standalone controller_manager; mujoco/gazebo/isaac host their
        # own. real is NOT here — Axol's CAN backend is deferred (no ros2_control plugin
        # yet), so sim.launch serves the sim backends only.
        standalone_cm_backends=frozenset({"mock"}),
        foxglove_params=DEFAULT_FOXGLOVE_PARAMS,
        # Axol MoveIt config is authored in-repo (Humble, bimanual 2x7-DOF).
        moveit_pkg="fm_bringup",
        moveit_cfg=os.path.join("config", "axol"),
        servo_config="servo.yaml",
        bringup_srdf={"axol": "axol.srdf"},
        moveit_srdf="axol.srdf",
        # Second servo_node for the left arm: its own servo.yaml + delta topics under
        # /servo_node_left/ (the primary servo_node drives the right arm).
        extra_servo_configs=(("servo_node_left", "servo_left.yaml"),),
        # Both arm controllers together claim all 14 revolute joints (the whole model),
        # so no extra joint_state_publisher is needed (unlike the G1-D's arm subset).
        full_state_jsp=False,
    ),
}


def get(robot_key):
    """Return the :class:`RobotSpec` for ``robot_key`` or raise a clear error."""
    try:
        return _ROBOTS[robot_key]
    except KeyError:
        raise RuntimeError(
            f"Unknown robot '{robot_key}'. Registered: {', '.join(sorted(_ROBOTS))}."
        )
