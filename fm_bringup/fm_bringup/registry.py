"""Robot registry for the fm_bringup control launch layer.

One entry per robot owns everything the control launch files vary on, so adding a
robot is a single :class:`RobotSpec` here instead of edits scattered across
``sim.launch.py`` / ``teleop.launch.py`` / ``servo.launch.py``. Mirrors
``fm_description``'s ``view_robot.launch.py``, which did the same for description
views.

Each entry carries:

    description   backend-selectable xacro + per-robot visual-mesh rewrite
    controllers   per-variant active/inactive sets + which backends need a
                  standalone controller_manager
    foxglove      foxglove_bridge params (mesh allowlist, buffer limits)
    servo         MoveIt Servo context: SRDF locator, MoveIt config package,
                  servo.yaml

The launch files read a spec via :func:`get` and call its helpers; they hold no
robot-specific data themselves.
"""

import os
import re
from dataclasses import dataclass
from typing import Optional

import xacro
from ament_index_python.packages import get_package_share_directory

# --- OpenArm specifics -------------------------------------------------------

# Visual meshes ship as z-up .stl under fm_description (converted at build); the
# upstream xacro points visuals at openarm_description .dae. Rewrite so Foxglove
# (fed by robot_state_publisher) renders. Collisions stay on openarm_description.
_OPENARM_MESH_RE = re.compile(r"package://openarm_description/([^\"']+?)\.dae")
_OPENARM_MESH_SUB = r"package://fm_description/openarm_meshes/\1.stl"

# foxglove_bridge params for the OpenArm: its package:// mesh paths run through a
# dotted directory (openarm_v2.0), which the default asset_uri_allowlist ([\w-]
# only) rejects, so nothing renders; [-\w.] admits the dot. send_buffer_limit is
# raised above the 10 MB default for the large default_bimanual body mesh.
_OPENARM_FOXGLOVE_PARAMS = {
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
    preset_arg: str  # xacro arg the variant maps to (e.g. "robot_preset")
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

    # --- path helpers --------------------------------------------------------

    def _config(self, *parts):
        return os.path.join(
            get_package_share_directory("fm_bringup"), "config", self.config_dir, *parts
        )

    def controllers_file(self, variant):
        return self._config(f"{variant}.controllers.yaml")

    def servo_params_file(self):
        return self._config(self.servo_config)

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
        mappings = {self.preset_arg: variant, "sim_backend": sim_backend}
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
        foxglove_params=_OPENARM_FOXGLOVE_PARAMS,
        moveit_pkg="openarm_bimanual_moveit_config",
        moveit_cfg=os.path.join("config", "openarm_v2.0"),
        servo_config="servo.yaml",
        bringup_srdf={"right_arm": "right_arm.srdf"},
        moveit_srdf="openarm_bimanual.srdf",
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
