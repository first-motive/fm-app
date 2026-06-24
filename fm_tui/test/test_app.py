"""fm_tui app tests: it mounts and composes its panels headless, themed or bare."""

import asyncio

from fm_tui.app import FmTuiApp
from fm_tui.theme import Header


def test_app_mounts_all_panels():
    async def go():
        async with FmTuiApp(connect_ros=False).run_test() as pilot:
            await pilot.pause()
            app = pilot.app
            assert app.query_one("#nodes") is not None
            assert app.query_one("#topics") is not None
            assert app.query_one("#rosout") is not None

    asyncio.run(go())


def test_header_shows_offline_until_ros_connects():
    # connect_ros=False means the graph never refreshes, so the header should
    # mount in its offline state and stay there.
    async def go():
        async with FmTuiApp(connect_ros=False).run_test() as pilot:
            await pilot.pause()
            assert pilot.app.query_one(Header)._connected is False

    asyncio.run(go())
