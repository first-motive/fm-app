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
from dataclasses import dataclass
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
_DEFAULT_FOXGLOVE_PARAMS = {
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
        foxglove_params=_DEFAULT_FOXGLOVE_PARAMS,
        moveit_pkg="openarm_bimanual_moveit_config",
        moveit_cfg=os.path.join("config", "openarm_v2.0"),
        servo_config="servo.yaml",
        bringup_srdf={"right_arm": "right_arm.srdf"},
        moveit_srdf="openarm_bimanual.srdf",
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
        foxglove_params=_DEFAULT_FOXGLOVE_PARAMS,
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
        label="Unitree G1-D",
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
        foxglove_params=_DEFAULT_FOXGLOVE_PARAMS,
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
        foxglove_params=_DEFAULT_FOXGLOVE_PARAMS,
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
