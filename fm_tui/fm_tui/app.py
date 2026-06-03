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
when it is installed, plain fallback twins otherwise. Live data comes from
``fm_tui.ros.RosBridge``, which spins a rclpy node on a background thread.
"""

from __future__ import annotations

from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Footer, Static

from fm_tui.theme import BorderedPanel, Header, LogView, apply_theme

# How often to refresh the node and topic lists, in seconds.
_GRAPH_REFRESH_SECONDS = 2.0


@apply_theme
class FmTuiApp(App):
    """The fm_tui monitor application."""

    TITLE = "fm_tui"
    BINDINGS = [("q", "quit", "Quit")]

    def __init__(self, connect_ros: bool = True, **kwargs) -> None:
        super().__init__(**kwargs)
        self._connect_ros = connect_ros
        self._ros = None

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
        self.query_one("#nodes", Static).update("(connecting…)")
        self.query_one("#topics", Static).update("(connecting…)")
        if not self._connect_ros:
            return
        # Import here so the app still constructs (and tests still run) without
        # a ROS2 environment on the path.
        from fm_tui.ros import RosBridge

        self._ros = RosBridge(self._emit_log)
        self._ros.start()
        self._refresh_graph()
        self.set_interval(_GRAPH_REFRESH_SECONDS, self._refresh_graph)

    def _emit_log(self, severity: str, message: str) -> None:
        # Called from the rclpy thread — hop back onto the app's thread.
        self.call_from_thread(self._write_log, severity, message)

    def _write_log(self, severity: str, message: str) -> None:
        self.query_one("#rosout", LogView).log_line(severity, message)

    def _refresh_graph(self) -> None:
        nodes = self._ros.nodes()
        topics = self._ros.topics()
        self.query_one("#nodes", Static).update("\n".join(nodes) or "(none)")
        self.query_one("#topics", Static).update("\n".join(topics) or "(none)")

    def on_unmount(self) -> None:
        if self._ros is not None:
            self._ros.shutdown()


def main() -> None:
    FmTuiApp().run()


if __name__ == "__main__":
    main()
