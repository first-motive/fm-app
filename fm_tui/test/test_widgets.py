"""Fallback-widget tests: the plain twins carry the FM palette and brand title.

These exercise ``fm_tui.widgets`` directly (not through the theme resolver), so
they cover the bare look regardless of whether nish-tui is installed.
"""

from fm_tui import palette
from fm_tui.widgets import BorderedPanel, Header, LogView


def test_border_title_is_uppercased():
    assert BorderedPanel(title="nodes").border_title == "NODES"


def test_panel_count_badge():
    panel = BorderedPanel(title="nodes")
    panel.set_count(12)
    assert panel.border_title == "NODES · 12"


def test_panel_empty_count_drops_badge():
    panel = BorderedPanel(title="nodes")
    panel.set_count(12)
    panel.set_count(0)
    assert panel.border_title == "NODES"


def test_header_live_status_shows_node_count():
    status = Header("fm_tui")._status_text(connected=True, node_count=12)
    assert "LIVE" in status.plain
    assert "12 nodes" in status.plain


def test_header_offline_status():
    status = Header("fm_tui")._status_text(connected=False, node_count=0)
    assert "OFFLINE" in status.plain


def test_log_line_colours_by_severity():
    line = LogView()._build_line("warn", "battery low")
    styles = " ".join(str(span.style) for span in line.spans)
    assert palette.AMBER in styles


def test_unknown_severity_falls_back_to_cream():
    line = LogView()._build_line("trace", "mystery")
    styles = " ".join(str(span.style) for span in line.spans)
    assert palette.CREAM in styles
