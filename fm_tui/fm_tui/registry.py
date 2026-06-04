"""Declarative registry — the single source of truth for the launcher menu.

The launcher (``fm_tui.launcher``) walks this structure to draw its menu and to
build the command it dispatches: action -> robot -> variant. Keeping the menu as
data, not control flow, means a new robot or variant is one entry here, not a new
screen branch.

Two kinds of action live side by side:

- **Wired** actions own a :class:`LaunchSpec`. The launcher runs
  ``ros2 launch <package> <launch_file> <robot_arg>:=<robot> <variant_arg>:=<variant>``
  for them. ``robot_description`` is the only wired action today; it converges on
  the same ``fm_description view_robot.launch.py`` that ``scripts/view-robot.sh``
  drives from the host, so the two entry points stay decoupled.
- **Stub** actions (``teleop``, ``autonomous``) carry ``launch=None`` and render
  disabled. They mark planned surface so the menu shape is stable before the
  launch graph exists.

Robot and variant lists mirror ``fm_description``'s ``view_robot.launch.py``
registry. The duplication is deliberate for v1 — the launch file owns dispatch
params, this file owns the menu — and is reconcilable later.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class LaunchSpec:
    """How a wired action turns a robot + variant into a ``ros2 launch`` call."""

    package: str
    launch_file: str
    robot_arg: str = "robot"
    variant_arg: str = "variant"

    def command(self, robot: str, variant: str) -> list[str]:
        """Build the ``ros2 launch`` argv for ``robot`` and ``variant``."""
        return [
            "ros2",
            "launch",
            self.package,
            self.launch_file,
            f"{self.robot_arg}:={robot}",
            f"{self.variant_arg}:={variant}",
        ]


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
class Action:
    """A top-level menu entry: wired (has a launch spec) or a stub."""

    key: str
    label: str
    launch: LaunchSpec | None = None
    robots: tuple[Robot, ...] = field(default_factory=tuple)

    @property
    def wired(self) -> bool:
        """True when this action can dispatch a launch."""
        return self.launch is not None


# Robot + variant lists mirror fm_description/launch/view_robot.launch.py.
_VIEW_ROBOT = LaunchSpec(package="fm_description", launch_file="view_robot.launch.py")

_ROBOTS = (
    Robot(
        key="g1_d",
        label="Unitree G1 (G1-D)",
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
            "right_arm",
            "left_arm",
            "default_bimanual",
            "right_arm_with_pinch_gripper",
            "left_arm_with_pinch_gripper",
        ),
        default_variant="right_arm",
    ),
)


ACTIONS: tuple[Action, ...] = (
    Action(
        key="robot_description",
        label="Robot Description",
        launch=_VIEW_ROBOT,
        robots=_ROBOTS,
    ),
    # Stubs — planned surface, no launch graph yet. launch=None renders disabled.
    Action(key="teleop", label="Teleop"),
    Action(key="autonomous", label="Autonomous"),
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
