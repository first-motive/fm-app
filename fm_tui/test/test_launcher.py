"""Launcher tests: menu builds from the registry, stubs never dispatch.

Headless — no ROS, no real launch. Dispatch is observed through the app's exit
value (the ``ros2 launch`` argv), never by running it.
"""

import asyncio

from textual.widgets import ListView

from fm_tui.launcher import FmLauncherApp
from fm_tui.registry import actions


def test_menu_builds_from_registry():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            assert len(menu) == len(actions())
            # First action is wired; the two stubs carry the disabled class.
            stub_count = sum("stub" in item.classes for item in menu.children)
            assert stub_count == 2

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


def test_stub_does_not_dispatch():
    async def go():
        async with FmLauncherApp().run_test() as pilot:
            await pilot.pause()
            menu = pilot.app.query_one("#menu", ListView)
            menu.index = 1  # teleop (stub)
            await pilot.press("enter")
            await pilot.pause()
            # No dispatch: app still running on the action level, no exit value.
            assert pilot.app.is_running
            assert pilot.app.return_value is None
            assert len(menu) == len(actions())

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
