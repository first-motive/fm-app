"""fm_tui launcher — an arrow-key menu that picks and dispatches a launch.

This is the launcher mode, a sibling to the ``fm_tui`` monitor (``app.py``). It
walks the declarative :mod:`fm_tui.registry` in three steps — action -> robot ->
variant — then dispatches the matching ``ros2 launch`` for wired actions::

    Header
    ┌ menu ───────────────────────────┐
    │ > Robot Description              │   ← action level
    │   Teleop            (not yet wired)
    │   Autonomous        (not yet wired)
    └──────────────────────────────────┘
    Footer   ↑↓ move · enter select · esc back · q quit

Dispatch handoff: selecting a variant exits the Textual app with the ``ros2
launch`` argv as its return value. :func:`main` then runs that command, so the
launch inherits the real terminal (the container entrypoint has already sourced
ROS + the overlay). Stub actions carry no launch spec; selecting one shows a
notice and never dispatches.

Widgets come from the theming layer (:mod:`fm_tui.theme`) so the launcher shares
the monitor's look, themed or bare.
"""

from __future__ import annotations

import subprocess

from textual.app import App, ComposeResult
from textual.widgets import Footer, Label, ListItem, ListView

from fm_tui.registry import Action, Robot, actions
from fm_tui.theme import BorderedPanel, Header, apply_theme

# Navigation levels, in walk order.
_ACTION, _ROBOT, _VARIANT = "action", "robot", "variant"


class _MenuItem(ListItem):
    """A list row carrying the registry object (or variant string) it selects."""

    def __init__(self, text: str, value: object, *, stub: bool = False) -> None:
        super().__init__(Label(text))
        self.value = value
        if stub:
            self.add_class("stub")


@apply_theme
class FmLauncherApp(App):
    """The fm_tui launcher: walk the registry, dispatch a launch."""

    TITLE = "fm_tui launcher"
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("escape", "back", "Back"),
    ]
    CSS = """
    .stub {
        color: $text-disabled;
    }
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._level = _ACTION
        self._action: Action | None = None
        self._robot: Robot | None = None

    def compose(self) -> ComposeResult:
        yield Header("fm_tui launcher — pick an action")
        with BorderedPanel(title="menu"):
            yield ListView(id="menu")
        yield Footer()

    def on_mount(self) -> None:
        self._rebuild()

    # --- menu construction -------------------------------------------------

    def _rebuild(self) -> None:
        """Repopulate the list for the current navigation level."""
        menu = self.query_one("#menu", ListView)
        menu.clear()
        for item in self._items_for_level():
            menu.append(item)
        menu.index = 0
        self._set_prompt()

    def _items_for_level(self) -> list[_MenuItem]:
        if self._level == _ACTION:
            return [
                _MenuItem(
                    a.label if a.wired else f"{a.label}  (not yet wired)",
                    a,
                    stub=not a.wired,
                )
                for a in actions()
            ]
        if self._level == _ROBOT:
            return [_MenuItem(r.label, r) for r in self._action.robots]
        return [
            _MenuItem(
                v if v != self._robot.default_variant else f"{v}  (default)",
                v,
            )
            for v in self._robot.variants
        ]

    def _set_prompt(self) -> None:
        header = self.query_one(Header)
        if self._level == _ACTION:
            header.update("fm_tui launcher — pick an action")
        elif self._level == _ROBOT:
            header.update(f"{self._action.label} — pick a robot")
        else:
            header.update(f"{self._robot.label} — pick a variant")

    # --- navigation --------------------------------------------------------

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        value = event.item.value
        if self._level == _ACTION:
            self._select_action(value)
        elif self._level == _ROBOT:
            self._robot = value
            self._level = _VARIANT
            self._rebuild()
        else:
            self._dispatch(value)

    def _select_action(self, act: Action) -> None:
        if not act.wired:
            self.notify(f"{act.label} is not yet wired.", severity="warning")
            return
        self._action = act
        self._level = _ROBOT
        self._rebuild()

    def action_back(self) -> None:
        """Step back one level; quit from the top."""
        if self._level == _VARIANT:
            self._level = _ROBOT
            self._robot = None
        elif self._level == _ROBOT:
            self._level = _ACTION
            self._action = None
        else:
            self.exit(None)
            return
        self._rebuild()

    # --- dispatch ----------------------------------------------------------

    def _dispatch(self, variant: str) -> None:
        """Exit with the launch argv; :func:`main` runs it post-teardown."""
        self.exit(self._action.launch.command(self._robot.key, variant))


def main() -> None:
    command = FmLauncherApp().run()
    if command:
        # The Textual UI has torn down; hand the terminal to the launch. The
        # container entrypoint already sourced ROS + the overlay, so the env is
        # ready and ros2 is on PATH.
        subprocess.run(command, check=False)


if __name__ == "__main__":
    main()
