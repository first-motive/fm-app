"""Declarative registry — the single source of truth for the launcher menu.

The launcher (``fm_tui.launcher``) walks this structure to draw its menu and to
build the command it dispatches: action -> (mode ->) robot -> variant. Keeping the
menu as data, not control flow, means a new robot or variant is one entry here, not
a new screen branch.

Three kinds of action live side by side:

- **Wired** actions own a :class:`LaunchSpec`. The launcher runs
  ``ros2 launch <package> <launch_file> <robot_arg>:=<robot> <variant_arg>:=<variant>``
  for them (``robot_description``, ``simulation``, ``data_capture``). ``robot_description``
  converges on the same ``fm_description view_robot.launch.py`` that ``scripts/view-robot.sh``
  drives from the host, so the two entry points stay decoupled.
- **Mode groups** carry ``launch=None`` but a non-empty ``modes`` tuple (``teleoperation``:
  Vision Mirror / Leader-Follower). The launcher inserts a mode-selection step; the chosen
  :class:`Mode` supplies the launch spec, robots, and backends from the robot step onward.
- **Stub** actions (``autonomous``) carry ``launch=None`` and no modes, and render disabled.
  They mark planned surface so the menu shape is stable before the launch graph exists.

Robot and variant lists mirror ``fm_description``'s ``view_robot.launch.py``
registry. The duplication is deliberate for v1 — the launch file owns dispatch
params, this file owns the menu — and is reconcilable later.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Field:
    """A free-text launch parameter collected from the operator before dispatch.

    Fielded actions (e.g. vision teleop, which needs a camera URL) add a form step after
    the menu walk. Each field becomes a ``name:=value`` launch argument; ``name`` must match
    a ``DeclareLaunchArgument`` in the target launch file.
    """

    name: str  # launch arg name (or config key when host_only)
    label: str  # prompt shown in the form
    default: str = ""
    required: bool = False  # non-empty value required to dispatch
    choices: tuple[str, ...] = ()  # if set, value must be one of these
    # Collected in the form and persisted for a host-side helper, but NOT passed
    # to ros2 launch. The camera picker uses this: the relay it selects can only
    # run on the host (see scripts/run/camera-bridge.sh), so the container launch
    # never sees it — command() skips host_only fields.
    host_only: bool = False
    # Conditional visibility: when set to ``(field_name, value)``, the launcher
    # shows this field only while the named sibling field holds ``value`` (e.g. the
    # phone IP appears only when Camera == "phone"). Purely a form-display hint —
    # command() and the export ignore it; a hidden field still carries its default.
    show_if: tuple[str, str] = ()


@dataclass(frozen=True)
class LaunchSpec:
    """How a wired action turns a robot + variant (+ backend + fields) into a launch call."""

    package: str
    launch_file: str
    robot_arg: str = "robot"
    variant_arg: str = "variant"
    # Sim/teleop actions also pick a backend; set this to pass it as a launch arg.
    backend_arg: str | None = None
    # Extra params collected via a form step and appended as name:=value (in this order).
    fields: tuple[Field, ...] = ()
    # Set on launches whose graph honours the viewer choice (robot_description).
    # When true, command() maps the persisted viewer to use_foxglove/use_rviz.
    # sim/teleop carry no rviz node, so they leave this false and ignore viewer.
    viewer_aware: bool = False

    @property
    def has_fields(self) -> bool:
        """True when the launcher should collect form fields before dispatch."""
        return bool(self.fields)

    def command(
        self,
        robot: str,
        variant: str,
        backend: str | None = None,
        params: dict[str, str] | None = None,
        viewer: str | None = None,
    ) -> list[str]:
        """Build the ``ros2 launch`` argv for ``robot``, ``variant`` (+ ``backend``, fields, ``viewer``)."""
        argv = [
            "ros2",
            "launch",
            self.package,
            self.launch_file,
            f"{self.robot_arg}:={robot}",
            f"{self.variant_arg}:={variant}",
        ]
        if self.backend_arg and backend:
            argv.append(f"{self.backend_arg}:={backend}")
        # Append fields in declaration order (deterministic argv) — collected value or default.
        # host_only fields (the camera picker) drive a host-side relay, not the launch, so
        # they never become launch args.
        params = params or {}
        for field_spec in self.fields:
            if field_spec.host_only:
                continue
            argv.append(f"{field_spec.name}:={params.get(field_spec.name, field_spec.default)}")
        if self.viewer_aware and viewer:
            # Pass both flags explicitly so the argv is unambiguous regardless of
            # the launch file's own defaults. foxglove is the standing default.
            # Only rviz turns the foxglove_bridge off and rviz on. Both foxglove
            # and panel keep the bridge up (:8765): panel is the fm_viewer browser
            # page over that same bridge, so it needs the bridge but launches no
            # Foxglove desktop app (that gating is host-side, not in this argv).
            use_rviz = viewer == "rviz"
            argv.append(f"use_foxglove:={'false' if use_rviz else 'true'}")
            argv.append(f"use_rviz:={'true' if use_rviz else 'false'}")
            # Joint control follows the viewer inside view_robot.launch.py
            # (use_jsp_gui defaults to auto -> jsp_gui on rviz, headless jsp on
            # foxglove), so no jsp flag is passed here. That keeps this argv
            # identical to FM Desktop's, which the parity test asserts.
        return argv


@dataclass(frozen=True)
class Robot:
    """A robot a wired action can target, with its selectable variants."""

    key: str
    label: str
    variants: tuple[str, ...]
    default_variant: str

    def __post_init__(self) -> None:
        if self.default_variant not in self.variants:
            raise ValueError(
                f"{self.key}: default_variant {self.default_variant!r} "
                f"not in variants {self.variants}"
            )


@dataclass(frozen=True)
class Mode:
    """A sub-action under an action that groups modes (e.g. Teleoperation ->
    Vision Mirror / Leader-Follower).

    A mode carries its own launch spec, robots, and backends — the same launch-bearing
    shape an :class:`Action` has. Once the operator picks a mode, the launcher treats it
    as the effective action from the robot step onward, so dispatch is identical to a
    mode-less action's.
    """

    key: str
    label: str
    launch: LaunchSpec | None = None
    robots: tuple[Robot, ...] = field(default_factory=tuple)
    backends: tuple[str, ...] = field(default_factory=tuple)

    @property
    def wired(self) -> bool:
        """True when this mode can dispatch a launch."""
        return self.launch is not None

    @property
    def has_backends(self) -> bool:
        """True when the launcher should add a backend selection step."""
        return bool(self.backends)


@dataclass(frozen=True)
class Action:
    """A top-level menu entry: wired (has a launch spec), a mode group, or a stub."""

    key: str
    label: str
    launch: LaunchSpec | None = None
    robots: tuple[Robot, ...] = field(default_factory=tuple)
    # Sim backends this action offers; empty means no backend step (robot -> variant
    # dispatches directly).
    backends: tuple[str, ...] = field(default_factory=tuple)
    # Optional mode level (action -> mode -> robot -> …). When set, the action is a group:
    # it carries no launch/robots of its own and the launcher inserts a mode-selection step.
    modes: tuple[Mode, ...] = field(default_factory=tuple)

    @property
    def wired(self) -> bool:
        """True when this action can dispatch a launch directly (no mode step)."""
        return self.launch is not None

    @property
    def has_backends(self) -> bool:
        """True when the launcher should add a backend selection step."""
        return bool(self.backends)

    @property
    def has_modes(self) -> bool:
        """True when the launcher should add a mode-selection step after the action."""
        return bool(self.modes)


# Robot + variant lists mirror fm_description/launch/view_robot.launch.py.
_VIEW_ROBOT = LaunchSpec(
    package="fm_description",
    launch_file="view_robot.launch.py",
    viewer_aware=True,
)

_ROBOTS = (
    Robot(
        key="g1_d",
        label="Unitree",
        variants=("g1_d", "g1_29dof_rev_1_0"),
        default_variant="g1_d",
    ),
    Robot(
        key="so101",
        label="LeRobot SO101",
        variants=("so101",),
        default_variant="so101",
    ),
    Robot(
        key="openarm",
        label="Enactic OpenArm",
        variants=(
            "default_bimanual",
            "right_arm_with_pinch_gripper",
            "left_arm_with_pinch_gripper",
            "right_arm",
            "left_arm",
        ),
        default_variant="default_bimanual",
    ),
    Robot(
        key="axol",
        label="Almond Bot Axol",
        variants=("bimanual",),
        default_variant="bimanual",
    ),
)


# Sim + teleop target the robots + presets that carry a controllers.yaml + Servo
# config (see fm_bringup/config/<robot> and fm_bringup.registry). Backends mirror
# sim.launch.py's sim_backend. The G1-D offers only its wheeled variant (the one with
# a right-arm control config); its real arm path is the arm_sdk bridge, not sim.launch.
_SIM_ROBOTS = (
    Robot(
        key="g1_d",
        label="Unitree",
        variants=("g1_d",),
        default_variant="g1_d",
    ),
    Robot(
        key="so101",
        label="LeRobot SO101",
        variants=("so101",),
        default_variant="so101",
    ),
    Robot(
        key="openarm",
        label="Enactic OpenArm",
        variants=("right_arm", "default_bimanual"),
        default_variant="right_arm",
    ),
    Robot(
        key="axol",
        label="Almond Bot Axol",
        variants=("axol",),
        default_variant="axol",
    ),
)
_SIM_BACKENDS = ("mujoco", "mock", "gazebo", "isaac")

_SIM = LaunchSpec(
    package="fm_bringup",
    launch_file="sim.launch.py",
    backend_arg="sim_backend",
)
_TELEOP = LaunchSpec(
    package="fm_bringup",
    launch_file="teleop.launch.py",
    backend_arg="sim_backend",
)

# Vision 1:1 hand-mirror teleop: one launch stands up sim + mirror teleop + recorder + reset
# (vision_session.launch.py). A few tuning knobs are collected in a form step; every non-
# host_only field name must match a vision_session launch arg. Mirror teleop is openarm/
# right_arm only today.
#
# Camera picker (host_only): the container reads its camera from a fixed host relay on
# :8090 (vision_session's camera_source default), so the launch args are identical for
# both sources. What differs is which host process feeds :8090 — the Mac's built-in camera
# (mac_camera_bridge.py) or a socat relay to the phone's IP-webcam. The container can start
# neither, so the choice + phone IP are persisted for the host manager (camera-bridge.sh)
# instead of passed to the launch. phone_ip is required only when camera == "phone"
# (enforced in the launcher).
_VISION_FIELDS = (
    Field("camera", "Camera", "phone", choices=("phone", "mac"), host_only=True),
    Field(
        "phone_ip",
        "Phone IP  (phone only, e.g. 192.168.1.207)",
        "",
        host_only=True,
        # Only relevant to the phone relay — hidden unless Camera == "phone". The operator
        # types just the LAN IP: the host relay (scripts/run/camera-bridge.sh) appends the
        # IP-webcam port (:8081) when none is given, so no port belongs here.
        show_if=("camera", "phone"),
    ),
    Field("rotate_deg", "Rotate clockwise (0/90/180/270)", "90"),
    Field("tracking_mode", "Tracking mode", "hand", choices=("hand", "full_body")),
    # publish_debug_image is intentionally not a form field: /vision/image (the skeleton
    # overlay the Foxglove/web viewers read) is always on for this session. vision_session
    # defaults the launch arg to true, so omitting it here keeps it published.
    # Run the arm with the pinch gripper (hand open/close drives the pinchers) or without it.
    Field("gripper", "Gripper (pinchers)", "off", choices=("on", "off")),
)
_VISION = LaunchSpec(
    package="fm_bringup",
    launch_file="vision_session.launch.py",
    backend_arg="sim_backend",
    fields=_VISION_FIELDS,
)
_VISION_ROBOTS = (
    Robot(
        key="openarm",
        label="Enactic OpenArm",
        variants=("right_arm",),
        default_variant="right_arm",
    ),
)
# macOS daily driver is mujoco; mock is the no-sim fallback. gazebo/isaac are Linux/GPU.
_VISION_BACKENDS = ("mujoco", "mock")


def _has_package(name: str) -> bool:
    """True when ``name`` is discoverable in the ament index.

    ``ament_index_python`` is imported lazily (and its absence tolerated) so the registry
    stays importable off a ROS environment — bare ``py_compile`` and the pure-Python tests
    never need it. Used to detect the optional ``fm_data`` learning overlay.
    """
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return False
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


# Teleoperation groups its inputs as modes: the 1:1 vision hand-mirror (sim + mirror teleop
# + recorder + reset, one launch) and the leader-follower device/browser path. Each mode keeps
# the exact launch spec, robots, and backends the old top-level actions carried, so behaviour is
# unchanged — only the menu shape moves them under one "Teleoperation" parent.
_VISION_MODE = Mode(
    key="vision_mirror",
    label="Vision Mirror",
    launch=_VISION,
    robots=_VISION_ROBOTS,
    backends=_VISION_BACKENDS,
)
_LEADER_FOLLOWER_MODE = Mode(
    key="leader_follower",
    label="Leader-Follower",
    launch=_TELEOP,
    robots=_SIM_ROBOTS,
    backends=_SIM_BACKENDS,
)

# Data Capture records a dataset. Today it drives the vision session with the recorder on
# (vision_session.launch.py — the `record` field defaults "true", so record:=true rides in the
# argv). Once the private learning overlay is installed it ships an `fm_data` package with
# dedicated dataset tooling; probe the ament index and swap the launch in when present,
# degrading to the vision session when absent. The fm_data launch name/args reconcile with the
# overlay when it lands.
_FM_DATA = LaunchSpec(
    package="fm_data",
    launch_file="record.launch.py",
    backend_arg="sim_backend",
    fields=_VISION_FIELDS,
)


def _data_capture_launch() -> LaunchSpec:
    """The Data Capture launch: fm_data dataset tooling when installed, else the vision session."""
    return _FM_DATA if _has_package("fm_data") else _VISION


ACTIONS: tuple[Action, ...] = (
    Action(
        key="robot_description",
        label="Robot Description",
        launch=_VIEW_ROBOT,
        robots=_ROBOTS,
    ),
    # Mode group — carries no launch of its own; the launcher inserts a mode step.
    Action(
        key="teleoperation",
        label="Teleoperation",
        modes=(_VISION_MODE, _LEADER_FOLLOWER_MODE),
    ),
    # Stub — planned surface, no launch graph yet. launch=None renders disabled.
    Action(key="autonomous", label="Autonomous"),
    Action(
        key="simulation",
        label="Simulation",
        launch=_SIM,
        robots=_SIM_ROBOTS,
        backends=_SIM_BACKENDS,
    ),
    Action(
        key="data_capture",
        label="Data Capture",
        launch=_data_capture_launch(),
        robots=_VISION_ROBOTS,
        backends=_VISION_BACKENDS,
    ),
)


def actions() -> tuple[Action, ...]:
    """Return all menu actions in display order."""
    return ACTIONS


def action(key: str) -> Action:
    """Look up an action by key; raise ``KeyError`` if absent."""
    for entry in ACTIONS:
        if entry.key == key:
            return entry
    raise KeyError(key)
