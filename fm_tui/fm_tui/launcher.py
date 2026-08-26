"""fm_tui launcher — an arrow-key menu that picks and dispatches a launch.

This is the launcher mode, a sibling to the ``fm_tui`` monitor (``app.py``). It
walks the declarative :mod:`fm_tui.registry` — action -> (mode ->) robot -> variant,
plus a backend step for sim/teleop specs — then dispatches the matching ``ros2 launch``
for wired actions::

    ◢ FIRST MOTIVE · ROBOT DESCRIPTION › G1_D
    ┏ MENU ────────────────────────────┓
    ┃ ▸ Robot Description               ┃   ← caret marks the highlighted row
    ┃   Teleoperation                   ┃   ← mode group: opens Vision Mirror / Leader-Follower
    ┃   Autonomous                      ┃   ← grey: stub, no launch graph yet
    ┃   Simulation                      ┃
    ┃   Data Capture                    ┃
    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
    Footer   [Q] QUIT  [ESC] BACK  [V] VIEWER: foxglove  [C] COMMS: zenoh

Dispatch handoff: selecting a variant exits the Textual app with the ``ros2
launch`` argv as its return value. :func:`main` then runs that command, so the
launch inherits the real terminal (the container entrypoint has already sourced
ROS + the overlay). Stub actions carry no launch spec; selecting one shows a
notice and never dispatches.

Viewer default: the ``V`` hotkey cycles the standing viewer (foxglove -> rviz ->
panel -> …), shown in the footer beside QUIT/BACK as ``VIEWER: <viewer>`` (its
binding label, refreshed on toggle). The choice persists through
:mod:`fm_tui.config` and rides into the ``robot_description`` dispatch as
``use_foxglove`` / ``use_rviz`` flags (panel keeps the bridge up like foxglove).

Transport: the ``C`` hotkey cycles this host's transport (zenoh <-> dds-lan),
shown as ``COMMS: <transport>``. Unlike the viewer it does not take effect in
this session — it records a request that ``./run.sh`` applies on the next launch,
because the transport is a fact on the machine's identity card and the launcher
usually runs inside a container that cannot write it. While a request is pending
the label reads ``COMMS: zenoh > dds-lan`` to say so.

Widgets come from the theming layer (:mod:`fm_tools.tui`) so the launcher shares
the monitor's look, themed or bare.
"""

from __future__ import annotations

import dataclasses
import os
import subprocess

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Button, Footer, Input, Label, ListItem, ListView, Select

from fm_tools.tui import BorderedPanel, Header, apply_theme
from fm_tools.tui.palette import LILAC, PLUM

from fm_tui import config
from fm_tui.registry import Action, Mode, Robot, actions

# Navigation levels, in walk order. Actions that group modes (teleoperation) add a mode
# step after the action; the chosen mode then supplies robots/backends/launch. Wired
# sim/backend actions add a backend step after the variant; robot_description dispatches
# straight from the variant. Fielded specs (vision) add a params form before dispatching.
_ACTION, _MODE, _ROBOT, _VARIANT, _BACKEND, _PARAMS, _PUB = (
    "action",
    "mode",
    "robot",
    "variant",
    "backend",
    "params",
    "pub",
)


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
        # Same shape as VIEWER, one step removed: its label carries the live
        # transport, and pressing it records a request rather than changing this
        # session (see action_toggle_transport).
        Binding("c", "toggle_transport", "COMMS"),
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
    /* Params form: a stacked label + Input per field, mounted in place of the menu. */
    .field-label {{
        color: $text-muted;
        margin: 1 0 0 0;
    }}
    #menu.hidden {{
        display: none;
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
        # Set only while walking a mode-grouping action (teleoperation); supplies the
        # effective launch/robots/backends from the robot step onward (see _source).
        self._mode: Mode | None = None
        self._robot: Robot | None = None
        self._variant: str | None = None
        self._backend: str | None = None
        # Mounted Input widgets (name -> Input), their labels (name -> Label), and every
        # form widget. All live only on _PARAMS; labels are tracked so conditional fields
        # (e.g. the phone IP) can hide their prompt as well as their input.
        self._field_inputs: dict[str, Input] = {}
        self._field_labels: dict[str, Label] = {}
        self._param_widgets: list = []
        # The standing viewer default, loaded from the persisted config. The `v`
        # binding flips and re-persists it; _dispatch reads it at launch time.
        self._viewer = config.get_viewer()
        # What this session is actually running on, and what has been asked for
        # instead. They are separate because a transport change cannot take
        # effect until run.sh has re-resolved it from the identity card.
        self._transport = config.active_transport()
        self._pending_transport = config.get_pending_transport()

    def compose(self) -> ComposeResult:
        yield Header()
        with BorderedPanel(title="menu"):
            yield ListView(id="menu")
        yield Footer()

    def on_mount(self) -> None:
        self._refresh_viewer_binding()
        self._refresh_transport_binding()
        self._rebuild()

    def _refresh_viewer_binding(self) -> None:
        """Show the current viewer in the footer's V key label (``VIEWER: <x>``)."""
        binding = self._bindings.keys.get("v")
        if binding is not None:
            self._bindings.keys["v"] = dataclasses.replace(
                binding, description=f"VIEWER: {self._viewer}"
            )
            self.refresh_bindings()

    def _refresh_transport_binding(self) -> None:
        """Show the transport in the footer's C key label (``COMMS: <x>``).

        A pending request is rendered as ``running > requested`` rather than as
        the requested value alone: showing only what was asked for would claim a
        change that has not happened yet, which is how an operator ends up
        recording a take on the middleware they thought they had left.
        """
        binding = self._bindings.keys.get("c")
        if binding is None:
            return
        label = self._transport
        if self._pending_transport is not None:
            label = f"{self._transport} > {self._pending_transport}"
        self._bindings.keys["c"] = dataclasses.replace(
            binding, description=f"COMMS: {label}"
        )
        self.refresh_bindings()

    # --- menu construction -------------------------------------------------

    @property
    def _source(self):
        """The launch-bearing object for the current walk: the chosen mode when the
        action groups modes, else the action itself. Robots, backends, the launch
        spec, and dispatch all read from here so a mode behaves like a plain action."""
        if self._action is not None and self._action.has_modes:
            return self._mode
        return self._action

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
            # A mode group (teleoperation) dispatches through its modes and a control
            # action (Record) publishes a topic, so neither is a stub even though its own
            # launch is None. Stubs read as disabled from the grey styling alone — no
            # "(not yet wired)" suffix needed.
            return [
                _MenuItem(a.label, a, stub=not a.wired and not a.has_modes and not a.is_pub)
                for a in actions()
            ]
        if self._level == _MODE:
            return [_MenuItem(m.label, m, stub=not m.wired) for m in self._action.modes]
        if self._level == _PUB:
            # A control action lists its labelled payloads (Start/Stop); selecting one
            # dispatches a one-shot ros2 topic pub.
            return [
                _MenuItem(label, payload) for label, payload in self._action.pub.options
            ]
        if self._level == _ROBOT:
            return [_MenuItem(r.label, r) for r in self._source.robots]
        if self._level == _VARIANT:
            # Show the friendly label when the robot defines one; the dispatched value
            # stays the raw variant key so ``variant:=<key>`` is unchanged.
            items = []
            for v in self._robot.variants:
                text = self._robot.variant_labels.get(v, v)
                if v == self._robot.default_variant:
                    text = f"{text}  (default)"
                items.append(_MenuItem(text, v))
            return items
        # _BACKEND: first backend is the default.
        return [
            _MenuItem(
                b if i != 0 else f"{b}  (default)",
                b,
            )
            for i, b in enumerate(self._source.backends)
        ]

    def _set_prompt(self) -> None:
        """Show the navigation trail as a header breadcrumb (action › robot › …)."""
        crumbs = [
            crumb
            for crumb in (
                self._action.label if self._action else None,
                self._mode.label if self._mode else None,
                self._robot.label if self._robot else None,
                self._variant,
                # Backend + a "params" marker only surface once the form is open.
                self._backend if self._level == _PARAMS else None,
                "params" if self._level == _PARAMS else None,
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
        elif self._level == _MODE:
            self._select_mode(value)
        elif self._level == _PUB:
            self._dispatch_pub(value)
        elif self._level == _ROBOT:
            self._robot = value
            self._level = _VARIANT
            self._rebuild()
        elif self._level == _VARIANT:
            # Sim/teleop pick a backend next; fielded specs collect a form; the rest dispatch.
            if self._source.has_backends:
                self._variant = value
                self._level = _BACKEND
                self._rebuild()
            elif self._source.launch.has_fields:
                self._variant = value
                self._enter_params()
            else:
                self._dispatch(value)
        else:  # _BACKEND
            # A fielded spec (vision) collects its form after the backend; others dispatch.
            if self._source.launch.has_fields:
                self._backend = value
                self._enter_params()
            else:
                self._dispatch(self._variant, value)

    def _select_action(self, act: Action) -> None:
        # A mode group opens its mode step; its own launch is None, so check modes first.
        if act.has_modes:
            self._action = act
            self._level = _MODE
            self._rebuild()
            return
        # A control action (Record) opens its option step (Start/Stop) — no launch, no robot.
        if act.is_pub:
            self._action = act
            self._level = _PUB
            self._rebuild()
            return
        if not act.wired:
            self.notify(f"{act.label} is not yet wired.", severity="warning")
            return
        self._action = act
        self._level = _ROBOT
        self._rebuild()

    def _select_mode(self, mode: Mode) -> None:
        if not mode.wired:
            self.notify(f"{mode.label} is not yet wired.", severity="warning")
            return
        self._mode = mode
        self._level = _ROBOT
        self._rebuild()

    def action_back(self) -> None:
        """Step back one level; quit from the top."""
        if self._level == _PARAMS:
            # Tear the form down and return to the backend step (or variant if backend-less).
            self._exit_params()
            if self._source.has_backends:
                self._level = _BACKEND
                self._backend = None
            else:
                self._level = _VARIANT
                self._variant = None
        elif self._level == _BACKEND:
            self._level = _VARIANT
            self._variant = None
        elif self._level == _VARIANT:
            self._level = _ROBOT
            self._robot = None
        elif self._level == _ROBOT:
            # Under a mode group, step back to the mode step; otherwise to the actions.
            if self._action.has_modes:
                self._level = _MODE
                self._mode = None
            else:
                self._level = _ACTION
                self._action = None
        elif self._level == _MODE:
            self._level = _ACTION
            self._action = None
        elif self._level == _PUB:
            self._level = _ACTION
            self._action = None
        else:
            self.exit(None)
            return
        self._rebuild()

    def action_toggle_transport(self) -> None:
        """Cycle the requested transport, relabel, and record it for run.sh.

        Deliberately does not change this session. The transport lives on the
        machine's identity card, this process usually runs inside a container
        that cannot write it, and a launch already in flight is on the middleware
        it started with either way. The request is parked in the launcher's own
        config — which sits on the mounted host repo root — and ``./run.sh``
        applies it through the same writer the ``--comms`` flag uses.
        """
        order = config.TRANSPORTS
        current = self._pending_transport or self._transport
        requested = order[(order.index(current) + 1) % len(order)]
        config.set_pending_transport(requested)
        self._pending_transport = config.get_pending_transport()
        self._refresh_transport_binding()
        if self._pending_transport is None:
            self.notify(f"transport stays {self._transport}")
        else:
            self.notify(
                f"transport {self._pending_transport} requested — "
                "restart with ./run.sh to apply it",
                severity="warning",
            )

    def action_toggle_viewer(self) -> None:
        """Cycle the viewer default (foxglove -> rviz -> panel -> …), relabel, persist."""
        order = config.VIEWERS
        # get_viewer() guarantees _viewer is a member, so index() is safe.
        self._viewer = order[(order.index(self._viewer) + 1) % len(order)]
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

    # --- params form -------------------------------------------------------

    def _initial_value(self, field_spec) -> str:
        """Prefill value for a form field: the field's declared default."""
        return field_spec.default

    def _enter_params(self) -> None:
        """Swap the menu for a stacked label+Input per field (mounted in the menu panel)."""
        self._level = _PARAMS
        menu = self.query_one("#menu", ListView)
        menu.add_class("hidden")
        self._field_inputs = {}
        self._field_labels = {}
        widgets: list = []
        for field_spec in self._source.launch.fields:
            label = field_spec.label + (" *" if field_spec.required else "")
            label_widget = Label(label, classes="field-label")
            self._field_labels[field_spec.name] = label_widget
            widgets.append(label_widget)
            initial = self._initial_value(field_spec)
            if field_spec.choices:
                # A dropdown for one-of fields — pick, don't type.
                widget = Select(
                    [(c, c) for c in field_spec.choices],
                    value=initial,
                    allow_blank=False,
                    id=f"field-{field_spec.name}",
                )
            else:
                widget = Input(value=initial, id=f"field-{field_spec.name}")
            self._field_inputs[field_spec.name] = widget
            widgets.append(widget)
        # An explicit submit control: several fields are Selects, and Enter on a
        # Select opens its dropdown rather than submitting. The button (and Enter in
        # any Input) both dispatch. Tab reaches it; it is the last widget so it sits
        # below the fields.
        widgets.append(Button("Launch", id="form-launch", variant="primary"))
        self._param_widgets = widgets
        # Mount into the bordered panel that holds the menu, after the (now hidden) ListView.
        menu.parent.mount(*widgets)
        self._apply_field_visibility()
        self._set_prompt()
        self.call_after_refresh(self._focus_first_field)

    def _apply_field_visibility(self) -> None:
        """Show/hide conditional fields against their controlling field's current value.

        A field with ``show_if=(name, value)`` is displayed only while the controlling
        field holds ``value``; its label and input are hidden together otherwise. Hidden
        fields keep their value, so the launch args in :meth:`_try_launch` are unaffected.
        (No field uses show_if today; the mechanism is retained for future forms.)
        """
        for field_spec in self._source.launch.fields:
            if not field_spec.show_if:
                continue
            ctrl_name, want = field_spec.show_if
            ctrl = self._field_inputs.get(ctrl_name)
            visible = ctrl is not None and str(ctrl.value) == want
            self._field_inputs[field_spec.name].display = visible
            label = self._field_labels.get(field_spec.name)
            if label is not None:
                label.display = visible

    def on_select_changed(self, event: Select.Changed) -> None:
        """Re-evaluate conditional field visibility when a controlling Select changes."""
        if self._level == _PARAMS:
            self._apply_field_visibility()

    def _focus_first_field(self) -> None:
        """Focus the first field so the form is immediately typeable."""
        for field_spec in self._source.launch.fields:
            self._field_inputs[field_spec.name].focus()
            return

    def on_input_submitted(self, event: Input.Submitted) -> None:
        """Enter in any Input submits the whole form (defaults are prefilled)."""
        if self._level == _PARAMS:
            self._try_launch()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """The form's Launch button submits — the path for when a Select is focused."""
        if self._level == _PARAMS and event.button.id == "form-launch":
            self._try_launch()

    def _try_launch(self) -> None:
        """Validate the fields and dispatch, or notify and stay on the form."""
        params: dict[str, str] = {}
        for field_spec in self._source.launch.fields:
            # Input.value is typed text; Select.value is the chosen option — both -> str.
            value = str(self._field_inputs[field_spec.name].value).strip()
            if field_spec.required and not value:
                self.notify(f"{field_spec.label} is required.", severity="warning")
                self._field_inputs[field_spec.name].focus()
                return
            if field_spec.choices and value not in field_spec.choices:
                self.notify(
                    f"{field_spec.label} must be one of: {', '.join(field_spec.choices)}.",
                    severity="warning",
                )
                self._field_inputs[field_spec.name].focus()
                return
            params[field_spec.name] = value
        self._dispatch(self._variant, self._backend, params)

    def _exit_params(self) -> None:
        """Remove the mounted form widgets and restore the menu list."""
        for widget in self._param_widgets:
            widget.remove()
        self._param_widgets = []
        self._field_inputs = {}
        self._field_labels = {}
        self.query_one("#menu", ListView).remove_class("hidden")

    # --- dispatch ----------------------------------------------------------

    def _dispatch(
        self,
        variant: str,
        backend: str | None = None,
        params: dict[str, str] | None = None,
    ) -> None:
        """Exit with the launch argv; :func:`main` runs it post-teardown."""
        self.exit(
            self._source.launch.command(
                self._robot.key, variant, backend, params, viewer=self._viewer
            )
        )

    def _dispatch_pub(self, value: str) -> None:
        """Exit with the one-shot ``ros2 topic pub`` argv for the chosen option; :func:`main`
        runs it post-teardown, the same path a launch dispatch takes."""
        self.exit(self._action.pub.command(value))


def main() -> None:
    command = FmLauncherApp().run()
    if command:
        # The Textual UI has torn down; hand the terminal to the launch. The
        # container entrypoint already sourced ROS + the overlay, so the env is
        # ready and ros2 is on PATH.
        try:
            subprocess.run(command, check=False)
        except KeyboardInterrupt:
            # Ctrl+C reaches the whole foreground process group: the launch
            # handles its own SIGINT and shuts down, so the launcher just exits
            # quietly instead of dumping a traceback over the teardown.
            pass


if __name__ == "__main__":
    main()
