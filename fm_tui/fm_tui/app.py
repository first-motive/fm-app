"""fm_tui — a single-screen ROS2 monitor.

Layout::

    Header
    ┌ nodes ──────┐ ┌ topics ─────┐
    │ ...         │ │ ...         │
    └─────────────┘ └─────────────┘
    ┌ /rosout ────────────────────┐
    │ severity-coloured log        │
    └──────────────────────────────┘
    Footer

Widgets come from the theming layer (``fm_tui.theme``): nish-tui's themed set
when it is installed, plain fallback twins otherwise. The app code is identical
either way.
"""

from __future__ import annotations

from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Footer, Static

from fm_tui.theme import BorderedPanel, Header, LogView, apply_theme


@apply_theme
class FmTuiApp(App):
    """The fm_tui monitor application."""

    TITLE = "fm_tui"
    BINDINGS = [("q", "quit", "Quit")]

    def compose(self) -> ComposeResult:
        yield Header("fm_tui — ROS2 monitor")
        with Horizontal():
            with BorderedPanel(title="nodes"):
                yield Static(id="nodes")
            with BorderedPanel(title="topics"):
                yield Static(id="topics")
        with BorderedPanel(title="/rosout"):
            yield LogView(id="rosout")
        yield Footer()

    def on_mount(self) -> None:
        # Placeholder content — replaced with live ROS2 data once rclpy is wired in.
        self.query_one("#nodes", Static).update("(no nodes yet)")
        self.query_one("#topics", Static).update("(no topics yet)")
        log = self.query_one("#rosout", LogView)
        for severity in ("debug", "info", "warn", "error"):
            log.log_line(severity, f"sample {severity} line")


def main() -> None:
    FmTuiApp().run()


if __name__ == "__main__":
    main()
