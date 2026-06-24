"""Launcher tests: menu builds from the registry, stubs never dispatch.

Headless — no ROS, no real launch. Dispatch is observed through the app's exit
value (the ``ros2 launch`` argv), never by running it.
"""

import asyncio

from textual.widgets import ListView

from textual.color import Color

from fm_tui import palette
from fm_tui.launcher import FmLauncherApp
from fm_tui.registry import actions
from fm_tui.theme import Header


def test_menu_builds_from_registry():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            assert len(menu) == len(actions())
            # Only autonomous remains a stub; it carries the disabled class.
            stub_count = sum("stub" in item.classes for item in menu.children)
            assert stub_count == 1

    asyncio.run(go())


def test_wired_path_dispatches_launch():
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
        ]

    asyncio.run(go())


def test_backend_path_dispatches_with_sim_backend():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = 3  # Simulation (last, wired, has backends)
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


def test_stub_does_not_dispatch():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = 2  # autonomous (stub)
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


def test_back_from_robot_returns_to_actions():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            await pilot.press("enter")  # robot_description -> robot level
            menu = pilot.app.query_one("#menu", ListView)
            assert len(menu) == 3  # g1_d, so101, openarm
            await pilot.press("escape")  # back to actions
            await pilot.pause()
            assert len(menu) == len(actions())

    asyncio.run(go())
