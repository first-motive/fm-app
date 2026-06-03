"""Theming layer — resolve which widget set fm_tui draws with.

nish-tui is a soft dependency. When it is installed, fm_tui borrows its themed
widgets and palette; when it is absent, fm_tui falls back to the plain twins in
``fm_tui.widgets``. Both sets share an API, so ``app.py`` imports its widgets
and ``apply_theme`` from here and never branches on availability itself.
"""

from __future__ import annotations

try:
    import nish_tui
    from nish_tui import BorderedPanel, Header, LogView

    HAS_NISH_TUI = True

    def apply_theme(app_cls):
        """Apply the nish-tui palette and stylesheet to ``app_cls``."""
        return nish_tui.apply(app_cls)

except ImportError:
    from fm_tui.widgets import BorderedPanel, Header, LogView

    HAS_NISH_TUI = False

    def apply_theme(app_cls):
        """No-op — the plain twins carry their own colours."""
        return app_cls


__all__ = ["BorderedPanel", "Header", "LogView", "apply_theme", "HAS_NISH_TUI"]
