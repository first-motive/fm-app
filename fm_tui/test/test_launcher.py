"""Launcher tests: menu builds from the registry, stubs never dispatch.

Headless — no ROS, no real launch. Dispatch is observed through the app's exit
value (the ``ros2 launch`` argv), never by running it.
"""

import asyncio

import pytest

from textual.widgets import Button, ListView

from textual.color import Color

from fm_tools.tui import Header, palette
from fm_tui import config
from fm_tui import launcher
from fm_tui.launcher import FmLauncherApp, main
from fm_tui.registry import _has_package, action, actions


async def _walk_to_vision_form(pilot):
    """Drive teleoperation -> Vision Mirror -> openarm -> right_arm -> mujoco -> params form."""
    menu = pilot.app.query_one("#menu", ListView)
    menu.index = [a.key for a in actions()].index("teleoperation")
    await pilot.press("enter")  # action -> mode (Vision Mirror, first)
    await pilot.press("enter")  # mode -> robot (openarm, only one)
    await pilot.press("enter")  # robot -> variant (right_arm)
    await pilot.press("enter")  # variant -> backend (mujoco default)
    await pilot.press("enter")  # backend -> params form
    await pilot.pause()


def test_variant_menu_shows_friendly_labels(monkeypatch, tmp_path):
    # The bimanual option reads as "Both arms (bimanual)", not the raw default_bimanual key.
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = [a.key for a in actions()].index("teleoperation")
            await pilot.press("enter")  # action -> mode (Vision Mirror, first)
            await pilot.press("enter")  # mode -> robot (openarm, only one)
            await pilot.press("enter")  # robot -> variant
            await pilot.pause()
            items = list(menu.children)
            assert [it._text for it in items] == [
                "Right arm only  (default)",
                "Both arms (bimanual)",
            ]
            # The dispatched value stays the raw variant key.
            assert [it.value for it in items] == ["right_arm", "default_bimanual"]

    asyncio.run(go())


def test_vision_form_dispatches_bimanual_variant(monkeypatch, tmp_path):
    # Selecting "Both arms" dispatches the raw variant key, not the friendly label.
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = [a.key for a in actions()].index("teleoperation")
            await pilot.press("enter")  # action -> mode (Vision Mirror, first)
            await pilot.press("enter")  # mode -> robot
            await pilot.press("enter")  # robot -> variant
            await pilot.pause()
            menu.index = 1  # Both arms (bimanual)
            await pilot.press("enter")  # variant -> backend (mujoco default)
            await pilot.press("enter")  # backend -> params form
            await pilot.pause()
            pilot.app.set_focus(pilot.app.query_one("#form-launch", Button))
            await pilot.press("enter")  # dispatch + exit
            await pilot.pause()
        assert "robot:=openarm" in pilot.app.return_value
        assert "variant:=default_bimanual" in pilot.app.return_value

    asyncio.run(go())


def test_menu_builds_from_registry():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            assert len(menu) == len(actions())
            # autonomous is the only stub, and only while the private policy layer
            # is absent; installed, it is wired like any other action.
            stub_count = sum("stub" in item.classes for item in menu.children)
            assert stub_count == (0 if _has_package("fm_policy_serve") else 1)

    asyncio.run(go())


def test_wired_path_dispatches_launch(monkeypatch, tmp_path):
    # Isolate the config so the default (foxglove) drives the dispatch regardless
    # of any .fm_tui.json in the working tree.
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            # robot_description (first) -> g1_d (first) -> g1_d default (first).
            await pilot.press("enter")  # action
            await pilot.press("enter")  # robot
            await pilot.press("enter")  # variant -> dispatch + exit
            await pilot.pause()
        assert pilot.app.return_value == [
            "ros2",
            "launch",
            "fm_description",
            "view_robot.launch.py",
            "robot:=g1_d",
            "variant:=g1_d",
            # Viewer default (foxglove) rides along as explicit launch flags.
            "use_foxglove:=true",
            "use_rviz:=false",
        ]

    asyncio.run(go())


def test_backend_path_dispatches_with_sim_backend(monkeypatch, tmp_path):
    # Isolate the config like the other dispatch tests; sim is viewer-unaware so
    # the argv stays free of viewer flags regardless of the persisted default.
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = [a.key for a in actions()].index("simulation")
            await pilot.press("enter")  # action -> robot (g1_d, first in the sim list)
            await pilot.press("enter")  # robot -> variant (g1_d)
            await pilot.press("enter")  # variant -> backend (mujoco default)
            await pilot.press("enter")  # backend -> dispatch + exit
            await pilot.pause()
        assert pilot.app.return_value == [
            "ros2",
            "launch",
            "fm_bringup",
            "sim.launch.py",
            "robot:=g1_d",
            "variant:=g1_d",
            "sim_backend:=mujoco",
        ]

    asyncio.run(go())


def test_vision_form_dispatches_with_defaults(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            await _walk_to_vision_form(pilot)
            # Submit via the Launch button (Enter on a Select opens its dropdown).
            pilot.app.set_focus(pilot.app.query_one("#form-launch", Button))
            await pilot.press("enter")  # button press -> dispatch + exit
            await pilot.pause()
        # Only the vision_session launch args ride the argv — the phone/webcam picker is gone.
        assert pilot.app.return_value == [
            "ros2",
            "launch",
            "fm_bringup",
            "vision_session.launch.py",
            "robot:=openarm",
            "variant:=right_arm",
            "sim_backend:=mujoco",
            "rotate_deg:=90",
            "tracking_mode:=hand",
            "capture_hands:=right",
            "camera_input:=device",
            "gripper:=off",
        ]

    asyncio.run(go())


def test_back_from_params_returns_to_backend():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            await _walk_to_vision_form(pilot)
            await pilot.press("escape")  # leave the form
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            # Back on the backend list (mujoco, mock), menu visible again.
            assert not menu.has_class("hidden")
            vision = next(m for m in action("teleoperation").modes if m.key == "vision_mirror")
            assert len(menu) == len(vision.backends)

    asyncio.run(go())


def test_record_action_dispatches_pub(monkeypatch, tmp_path):
    # Record is a control action: selecting it lists Start/Stop (no robot step), and
    # choosing one exits with a one-shot `ros2 topic pub` to /capture/record.
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = [a.key for a in actions()].index("record")
            await pilot.press("enter")  # action -> pub options (Start/Stop)
            await pilot.pause()
            # The option step lists the PubSpec options, not robots.
            assert [it._text for it in menu.children] == ["Start", "Stop"]
            await pilot.press("enter")  # Start -> dispatch + exit
            await pilot.pause()
        assert pilot.app.return_value == [
            "ros2", "topic", "pub", "--times", "3", "--rate", "5",
            "/capture/record", "std_msgs/msg/Bool", "{data: true}",
        ]

    asyncio.run(go())


def test_record_stop_dispatches_false(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = [a.key for a in actions()].index("record")
            await pilot.press("enter")  # -> pub options
            await pilot.pause()
            menu.index = 1  # Stop
            await pilot.press("enter")  # dispatch + exit
            await pilot.pause()
        assert pilot.app.return_value[-1] == "{data: false}"

    asyncio.run(go())


def test_back_from_pub_returns_to_actions():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = [a.key for a in actions()].index("record")
            await pilot.press("enter")  # -> pub options (Start/Stop)
            await pilot.pause()
            assert len(menu) == 2
            await pilot.press("escape")  # back to the action list
            await pilot.pause()
            assert len(menu) == len(actions())

    asyncio.run(go())


@pytest.mark.skipif(
    _has_package("fm_policy_serve"),
    reason="autonomous is wired when the policy layer is installed, so no stub remains",
)
def test_stub_does_not_dispatch():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = [a.key for a in actions()].index("autonomous")
            await pilot.press("enter")
            await pilot.pause()
            # No dispatch: app still running on the action level, no exit value.
            assert pilot.app.is_running
            assert pilot.app.return_value is None
            assert len(menu) == len(actions())

    asyncio.run(go())


def test_first_row_highlighted_after_each_level():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            # Action level: first row highlighted without any arrow key.
            assert menu.index == 0
            assert menu.highlighted_child is menu.children[0]
            await pilot.press("enter")  # into robot level
            await pilot.pause()
            # Robot level: first row highlighted immediately on entry.
            assert menu.index == 0
            assert menu.highlighted_child is menu.children[0]

    asyncio.run(go())


def test_selected_row_highlight_is_plum_not_blue():
    # Guards the recurring "selected row is blue" bug: the highlight CSS must
    # repaint the row background plum, overriding Textual's blue accent.
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            highlighted = [
                row
                for row in menu.children
                if row.has_class("-highlight") or row.has_class("--highlight")
            ]
            assert highlighted, "no highlighted row"
            assert highlighted[0].styles.background == Color.parse(palette.PLUM)

    asyncio.run(go())


def test_caret_marks_only_the_highlighted_row():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            assert menu.children[0]._display.startswith("▸")
            assert menu.children[1]._display.startswith("  ")

    asyncio.run(go())


def test_header_breadcrumb_tracks_navigation():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            header = pilot.app.query_one(Header)
            # Action level: no trail yet.
            assert header._title == ""
            await pilot.press("enter")  # robot_description -> robot level
            await pilot.pause()
            assert header._title == actions()[0].label

    asyncio.run(go())


def test_toggle_flips_persists_and_relabels(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))

    async def go():
        app = FmLauncherApp()
        async with app.run_test() as pilot:
            await pilot.pause()
            assert app._viewer == "foxglove"
            # The footer V key carries the current viewer.
            assert app._bindings.keys["v"].description == "VIEWER: foxglove"
            await pilot.press("v")
            await pilot.pause()
            assert app._viewer == "rviz"
            # The footer label follows the cycle.
            assert app._bindings.keys["v"].description == "VIEWER: rviz"
            # V cycles all three (foxglove -> rviz -> panel -> foxglove).
            await pilot.press("v")
            await pilot.pause()
            assert app._viewer == "panel"
            await pilot.press("v")
            await pilot.pause()
            assert app._viewer == "foxglove"
        # The last flip is persisted.
        assert config.get_viewer() == "foxglove"

    asyncio.run(go())


def test_dispatch_carries_rviz_when_default(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    config.set_viewer("rviz")

    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            await pilot.press("enter")  # robot_description
            await pilot.press("enter")  # g1_d
            await pilot.press("enter")  # variant -> dispatch
            await pilot.pause()
        # rviz rides in as its viewer flags; the launch derives jsp_gui from
        # use_rviz, so no jsp flag is passed here.
        assert pilot.app.return_value[-2:] == ["use_foxglove:=false", "use_rviz:=true"]

    asyncio.run(go())


def test_macos_toggle_to_rviz_warns(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    monkeypatch.setenv("FM_HOST_OS", "macos")

    async def go():
        app = FmLauncherApp()
        warnings = []
        async with app.run_test() as pilot:
            monkeypatch.setattr(
                app, "notify", lambda message, **kw: warnings.append((message, kw))
            )
            await pilot.press("v")  # foxglove -> rviz on macOS
            await pilot.pause()
        assert app._viewer == "rviz"  # still flips and persists
        assert warnings, "expected a macOS rviz warning"
        assert warnings[0][1].get("severity") == "warning"

    asyncio.run(go())


def test_macos_toggle_off_rviz_is_silent(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    monkeypatch.setenv("FM_HOST_OS", "macos")
    config.set_viewer("rviz")

    async def go():
        app = FmLauncherApp()
        warnings = []
        async with app.run_test() as pilot:
            monkeypatch.setattr(
                app, "notify", lambda message, **kw: warnings.append((message, kw))
            )
            await pilot.press("v")  # rviz -> panel: no warning (only entering rviz warns)
            await pilot.pause()
        assert app._viewer == "panel"
        assert not warnings

    asyncio.run(go())


def test_back_from_robot_returns_to_actions():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            await pilot.press("enter")  # robot_description -> robot level
            menu = pilot.app.query_one("#menu", ListView)
            assert len(menu) == 4  # g1_d, so101, openarm, axol
            await pilot.press("escape")  # back to actions
            await pilot.pause()
            assert len(menu) == len(actions())

    asyncio.run(go())


def test_main_absorbs_launch_ctrl_c(monkeypatch):
    # Ctrl+C during the launch hits the whole process group and surfaces in the
    # launcher as KeyboardInterrupt from subprocess.run. main() must swallow it
    # so teardown stays quiet instead of dumping a traceback.
    monkeypatch.setattr(launcher.FmLauncherApp, "run", lambda self: ["ros2", "launch"])

    def raise_ctrl_c(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(launcher.subprocess, "run", raise_ctrl_c)
    main()  # must return, not raise
