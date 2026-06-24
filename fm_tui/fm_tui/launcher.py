"""fm_tui launcher — an arrow-key menu that picks and dispatches a launch.

This is the launcher mode, a sibling to the ``fm_tui`` monitor (``app.py``). It
walks the declarative :mod:`fm_tui.registry` — action -> robot -> variant, plus a
backend step for sim/teleop actions — then dispatches the matching ``ros2 launch``
for wired actions::

    ◢ FIRST MOTIVE · ROBOT DESCRIPTION › G1_D
    ┏ MENU ────────────────────────────┓
    ┃ ▸ Robot Description               ┃   ← caret marks the highlighted row
    ┃   Teleop            (not yet wired)┃
    ┃   Autonomous        (not yet wired)┃
    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
    Footer   [Q] QUIT   [ESC] BACK

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

from fm_tui.palette import LILAC, PLUM
from fm_tui.registry import Action, Robot, actions
from fm_tui.theme import BorderedPanel, Header, apply_theme

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
    ]
    CSS = f"""
    .stub {{
        color: $text-disabled;
    }}
    /* Override the highlight cursor bar (Textual default, and any theme such as
       nish-tui, paint it with the accent — often bright blue). Match :focus to
       win specificity (the menu always holds focus); !important beats the theme
       variable; the Label rule recolours the row text, which Label sets itself. */
    ListView:focus > ListItem.-highlight,
    ListView > ListItem.-highlight {{
        background: {PLUM} !important;
        color: {LILAC} !important;
        text-style: bold;
    }}
    ListView:focus > ListItem.-highlight Label,
    ListView > ListItem.-highlight Label {{
        color: {LILAC} !important;
    }}
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._level = _ACTION
        self._action: Action | None = None
        self._robot: Robot | None = None
        self._variant: str | None = None

    def compose(self) -> ComposeResult:
        yield Header()
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

    # --- dispatch ----------------------------------------------------------

    def _dispatch(self, variant: str, backend: str | None = None) -> None:
        """Exit with the launch argv; :func:`main` runs it post-teardown."""
        self.exit(self._action.launch.command(self._robot.key, variant, backend))


def main() -> None:
    command = FmLauncherApp().run()
    if command:
        # The Textual UI has torn down; hand the terminal to the launch. The
        # container entrypoint already sourced ROS + the overlay, so the env is
        # ready and ros2 is on PATH.
        subprocess.run(command, check=False)


if __name__ == "__main__":
    main()
