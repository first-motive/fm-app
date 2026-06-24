"""Plain fallback widgets — fm_tui's look when nish-tui is not installed.

These twins mirror the nish-tui widget API (``Header(title)``,
``BorderedPanel(..., title=)``, ``LogView.log_line(severity, message)``) so the
theming layer can swap one set for the other without touching the app. They
carry the First Motive palette (:mod:`fm_tui.palette`), so fm_tui stays on-brand
and readable even bare.
"""

from __future__ import annotations

from rich.table import Table
from rich.text import Text
from textual.containers import Container
from textual.widgets import RichLog, Static

from fm_tui.palette import CREAM, LILAC, PLUM, SAND, SEVERITY


class Header(Static):
    """Branded two-zone status bar: brand mark left, live ROS status right.

    Drawn as a full-width grid — ``◢ FIRST MOTIVE · <title>`` on the left, a
    connection/node-count readout on the right. ``update(title)`` rewrites the
    title (the launcher's breadcrumb); ``set_status`` paints the right zone. The
    right zone stays blank until ``set_status`` is first called, so the launcher
    (no ROS link) shows a bare brand bar.
    """

    DEFAULT_CSS = f"""
    Header {{
        background: {PLUM};
        height: 1;
        padding: 0 1;
    }}
    """

    def __init__(self, title: str = "", **kwargs) -> None:
        super().__init__(**kwargs)
        self._title = title
        self._connected: bool | None = None
        self._node_count = 0
        self._render_bar()

    def update(self, title: str = "") -> None:
        """Rewrite the title zone (used by the launcher breadcrumb)."""
        self._title = title
        self._render_bar()

    def set_status(self, connected: bool, node_count: int = 0) -> None:
        """Paint the right zone: ``● LIVE · N nodes`` or ``○ OFFLINE``."""
        self._connected = connected
        self._node_count = node_count
        self._render_bar()

    def _brand_text(self) -> Text:
        brand = Text()
        brand.append("◢ ", style=LILAC)
        brand.append("FIRST MOTIVE", style=f"bold {CREAM}")
        if self._title:
            brand.append(f" · {self._title.upper()}", style=SAND)
        return brand

    def _status_text(self, connected: bool, node_count: int) -> Text:
        status = Text()
        if connected:
            status.append("ROS2 ", style=SAND)
            status.append("● ", style=LILAC)
            status.append(f"LIVE · {node_count} nodes", style=CREAM)
        else:
            status.append("ROS2 ○ OFFLINE", style=f"dim {SAND}")
        return status

    def _render_bar(self) -> None:
        right = Text() if self._connected is None else self._status_text(
            self._connected, self._node_count
        )
        bar = Table.grid(expand=True)
        bar.add_column(justify="left")
        bar.add_column(justify="right")
        bar.add_row(self._brand_text(), right)
        super().update(bar)


class BorderedPanel(Container):
    """Titled container with a heavy lilac border and an uppercase brand title."""

    DEFAULT_CSS = f"""
    BorderedPanel {{
        border: heavy {LILAC};
        border-title-color: {SAND};
        height: auto;
        padding: 0 1;
    }}
    """

    def __init__(self, *children, title: str = "", **kwargs) -> None:
        super().__init__(*children, **kwargs)
        self._base_title = title.upper()
        self.border_title = self._base_title

    def set_count(self, count: int) -> None:
        """Badge the title with a live count (``NODES · 12``); 0 drops the badge."""
        self.border_title = f"{self._base_title} · {count}" if count else self._base_title


class LogView(RichLog):
    """Scrolling log; colours lines by severity from the FM palette."""

    DEFAULT_CSS = """
    LogView {
        height: 1fr;
    }
    """

    def __init__(self, **kwargs) -> None:
        kwargs.setdefault("markup", False)
        kwargs.setdefault("wrap", True)
        super().__init__(**kwargs)

    def _build_line(self, severity: str, message: str) -> Text:
        _glyph, colour = SEVERITY.get(severity.lower(), ("·", CREAM))
        line = Text()
        line.append(f"{severity.upper():<5} ", style=f"bold {colour}")
        line.append(message, style=colour)
        return line

    def log_line(self, severity: str, message: str) -> None:
        self.write(self._build_line(severity, message))
