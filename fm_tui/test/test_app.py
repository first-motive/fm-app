"""fm_tui app tests: it mounts and composes its panels headless, themed or bare."""

import asyncio

from fm_tui.app import FmTuiApp


def test_app_mounts_all_panels():
    async def go():
        async with FmTuiApp().run_test() as pilot:
            await pilot.pause()
            app = pilot.app
            assert app.query_one("#nodes") is not None
            assert app.query_one("#topics") is not None
            assert app.query_one("#rosout") is not None

    asyncio.run(go())
