"""fm_tui launcher — an arrow-key menu that picks and dispatches a launch.

This is the launcher mode, a sibling to the ``fm_tui`` monitor (``app.py``). It
walks the declarative :mod:`fm_tui.registry` — action -> robot -> variant, plus a
backend step for sim/teleop actions — then dispatches the matching ``ros2 launch``
for wired actions::

    ◢ FIRST MOTIVE · ROBOT DESCRIPTION › G1_D
    ┏ MENU ────────────────────────────┓
    ┃ ▸ Robot Description               ┃   ← caret marks the highlighted row
    ┃   Teleop                          ┃
    ┃   Autonomous                      ┃   ← grey: stub, no launch graph yet
    ┃   Simulation                      ┃
    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
    Footer   [Q] QUIT   [ESC] BACK   [V] VIEWER: foxglove

Dispatch handoff: selecting a variant exits the Textual app with the ``ros2
launch`` argv as its return value. :func:`main` then runs that command, so the
launch inherits the real terminal (the container entrypoint has already sourced
ROS + the overlay). Stub actions carry no launch spec; selecting one shows a
notice and never dispatches.

Viewer default: the ``V`` hotkey flips the standing viewer (Foxglove ⇄ rviz),
shown in the footer beside QUIT/BACK as ``VIEWER: <viewer>`` (its binding label,
refreshed on toggle). The choice persists through :mod:`fm_tui.config` and rides
into the ``robot_description`` dispatch as ``use_foxglove`` / ``use_rviz`` flags.

Widgets come from the theming layer (:mod:`fm_tools.tui`) so the launcher shares
the monitor's look, themed or bare.
"""

from __future__ import annotations

import dataclasses
import os
import subprocess

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, Label, ListItem, ListView

from fm_tools.tui import BorderedPanel, Header, apply_theme
from fm_tools.tui.palette import LILAC, PLUM

from fm_tui import config
from fm_tui.registry import Action, Robot, actions

# Navigation levels, in walk order. Wired sim/teleop actions add a backend step
# after the variant; robot_description dispatches straight from the variant.
_ACTION, _ROBOT, _VARIANT, _BACKEND = "action", "robot", "variant", "backend"


class _MenuItem(ListItem):
    """A list row carrying the registry object (or variant string) it selects.

    The row reserves a two-column gutter for the selection caret so highlighting
    a row does not shift its text; :meth:`set_selected` fills the gutter.
    """

    def __init__(self, text: str, value: object, *, stub: bool = False) -> None:
        self._text = text
        self._display = f"  {text}"
        self._label = Label(self._display)
        super().__init__(self._label)
        self.value = value
        if stub:
            self.add_class("stub")

    def set_selected(self, selected: bool) -> None:
        """Show the ``▸`` caret on the highlighted row, blank gutter otherwise."""
        self._display = f"{'▸' if selected else ' '} {self._text}"
        self._label.update(self._display)


@apply_theme
class FmLauncherApp(App):
    """The fm_tui launcher: walk the registry, dispatch a launch."""

    TITLE = "fm_tui launcher"
    BINDINGS = [
        ("q", "quit", "QUIT"),
        ("escape", "back", "BACK"),
        # Sits in the footer beside QUIT/BACK; its label carries the live viewer
        # (VIEWER: <viewer>), refreshed on toggle via _refresh_viewer_binding.
        Binding("v", "toggle_viewer", "VIEWER"),
    ]
    CSS = f"""
    Screen {{
        padding: 1 2;
    }}
    /* Wrap the menu to its rows instead of stretching (ListView defaults to
       height: 1fr); cap at the viewport so a long list scrolls inside the box
       rather than pushing it past the terminal bottom. */
    #menu {{
        height: auto;
        max-height: 100%;
    }}
    .stub {{
        color: $text-disabled;
    }}
    /* Recolour the selected-row highlight (Textual paints it with the blue
       accent). Textual renamed the class from `--highlight` (<=0.8x) to
       `-highlight` (>=0.86), so cover both spellings, focused and blurred. The
       Label rule recolours the row text, which the Label sets on itself. */
    ListView > ListItem.--highlight,
    ListView:focus > ListItem.--highlight,
    ListView > ListItem.-highlight,
    ListView:focus > ListItem.-highlight {{
        background: {PLUM} !important;
        color: {LILAC} !important;
        text-style: bold;
    }}
    ListView > ListItem.--highlight Label,
    ListView:focus > ListItem.--highlight Label,
    ListView > ListItem.-highlight Label,
    ListView:focus > ListItem.-highlight Label {{
        color: {LILAC} !important;
    }}
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._level = _ACTION
        self._action: Action | None = None
        self._robot: Robot | None = None
        self._variant: str | None = None
        # The standing viewer default, loaded from the persisted config. The `v`
        # binding flips and re-persists it; _dispatch reads it at launch time.
        self._viewer = config.get_viewer()

    def compose(self) -> ComposeResult:
        yield Header()
        with BorderedPanel(title="menu"):
            yield ListView(id="menu")
        yield Footer()

    def on_mount(self) -> None:
        self._refresh_viewer_binding()
        self._rebuild()

    def _refresh_viewer_binding(self) -> None:
        """Show the current viewer in the footer's V key label (``VIEWER: <x>``)."""
        binding = self._bindings.keys.get("v")
        if binding is not None:
            self._bindings.keys["v"] = dataclasses.replace(
                binding, description=f"VIEWER: {self._viewer}"
            )
            self.refresh_bindings()

    # --- menu construction -------------------------------------------------

    def _rebuild(self) -> None:
        """Repopulate the list for the current navigation level."""
        menu = self.query_one("#menu", ListView)
        menu.clear()
        for item in self._items_for_level():
            menu.append(item)
        self._set_prompt()
        # Appended items mount on the next refresh; setting the index now would
        # land before they exist, so the first row paints unhighlighted until an
        # arrow key re-evaluates it. Defer the highlight (and focus) past the
        # mount so the first row is selected and visible straight away.
        self.call_after_refresh(self._highlight_first, menu)

    def _highlight_first(self, menu: ListView) -> None:
        """Select and show the first row once the new items have mounted."""
        # Force a Highlighted event even when the index is already 0 (clearing
        # to None first), then take focus so the highlight is drawn.
        menu.index = None
        menu.index = 0
        menu.focus()

    def _items_for_level(self) -> list[_MenuItem]:
        if self._level == _ACTION:
            # Stub actions read as disabled from the grey styling alone — no
            # "(not yet wired)" suffix needed.
            return [_MenuItem(a.label, a, stub=not a.wired) for a in actions()]
        if self._level == _ROBOT:
            return [_MenuItem(r.label, r) for r in self._action.robots]
        if self._level == _VARIANT:
            return [
                _MenuItem(
                    v if v != self._robot.default_variant else f"{v}  (default)",
                    v,
                )
                for v in self._robot.variants
            ]
        # _BACKEND: first backend is the default.
        return [
            _MenuItem(
                b if i != 0 else f"{b}  (default)",
                b,
            )
            for i, b in enumerate(self._action.backends)
        ]

    def _set_prompt(self) -> None:
        """Show the navigation trail as a header breadcrumb (action › robot › …)."""
        crumbs = [
            crumb
            for crumb in (
                self._action.label if self._action else None,
                self._robot.label if self._robot else None,
                self._variant,
            )
            if crumb
        ]
        self.query_one(Header).update(" › ".join(crumbs))

    # --- navigation --------------------------------------------------------

    def on_list_view_highlighted(self, event: ListView.Highlighted) -> None:
        """Move the ``▸`` caret to the highlighted row."""
        for item in self.query_one("#menu", ListView).children:
            if isinstance(item, _MenuItem):
                item.set_selected(item is event.item)

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        value = event.item.value
        if self._level == _ACTION:
            self._select_action(value)
        elif self._level == _ROBOT:
            self._robot = value
            self._level = _VARIANT
            self._rebuild()
        elif self._level == _VARIANT:
            # Sim/teleop pick a backend next; everything else dispatches now.
            if self._action.has_backends:
                self._variant = value
                self._level = _BACKEND
                self._rebuild()
            else:
                self._dispatch(value)
        else:
            self._dispatch(self._variant, value)

    def _select_action(self, act: Action) -> None:
        if not act.wired:
            self.notify(f"{act.label} is not yet wired.", severity="warning")
            return
        self._action = act
        self._level = _ROBOT
        self._rebuild()

    def action_back(self) -> None:
        """Step back one level; quit from the top."""
        if self._level == _BACKEND:
            self._level = _VARIANT
            self._variant = None
        elif self._level == _VARIANT:
            self._level = _ROBOT
            self._robot = None
        elif self._level == _ROBOT:
            self._level = _ACTION
            self._action = None
        else:
            self.exit(None)
            return
        self._rebuild()

    def action_toggle_viewer(self) -> None:
        """Flip the viewer default, refresh the footer label, and persist it."""
        self._viewer = "rviz" if self._viewer == "foxglove" else "foxglove"
        config.set_viewer(self._viewer)
        self._refresh_viewer_binding()
        # rviz has no native macOS build; on a Mac it renders in the container and
        # streams to the browser over VNC (run.sh opens it) with software GL —
        # slower than Foxglove. Inform, but still persist: the choice is valid.
        if self._viewer == "rviz" and os.environ.get("FM_HOST_OS") == "macos":
            self.notify(
                "rviz on macOS opens in the browser over VNC (software GL — Foxglove is faster).",
                severity="warning",
            )

    # --- dispatch ----------------------------------------------------------

    def _dispatch(self, variant: str, backend: str | None = None) -> None:
        """Exit with the launch argv; :func:`main` runs it post-teardown."""
        self.exit(
            self._action.launch.command(
                self._robot.key, variant, backend, viewer=self._viewer
            )
        )


def main() -> None:
    command = FmLauncherApp().run()
    if command:
        # The Textual UI has torn down; hand the terminal to the launch. The
        # container entrypoint already sourced ROS + the overlay, so the env is
        # ready and ros2 is on PATH.
        subprocess.run(command, check=False)


if __name__ == "__main__":
    main()
