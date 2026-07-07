"""fm_tui config — persist operator preferences to a small JSON file.

The launcher remembers the chosen viewer (Foxglove or rviz) across runs instead
of re-asking on every launch. The preference lives in a JSON dict so later keys
(default robot, default backend) can join without a format change.

Path resolution::

    FM_TUI_CONFIG   when set, the exact file to read/write
    else            .fm_tui.json in the current working directory

``run.sh`` sets ``FM_TUI_CONFIG`` to the container's ``/ws/.fm_tui.json`` — the
mounted host repo root, the one path that survives a container teardown. Outside
that mount the cwd fallback keeps the module usable in tests and ad-hoc runs.

A missing or unreadable file yields the defaults, so the first launch always has
a valid viewer and never crashes on a fresh checkout.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

# The one preference v1 persists. Kept as a dict so more keys can join later.
_DEFAULTS = {"viewer": "foxglove"}

# The viewers the launcher can dispatch. get_viewer() falls back to the default
# for any value outside this set, so a hand-edited config can never wedge the UI.
VIEWERS = ("foxglove", "rviz")


def config_path() -> Path:
    """Resolve the config file: ``FM_TUI_CONFIG`` if set, else ``cwd/.fm_tui.json``."""
    override = os.environ.get("FM_TUI_CONFIG")
    return Path(override) if override else Path.cwd() / ".fm_tui.json"


def load() -> dict:
    """Read the config, merged over the defaults; return the defaults if unreadable."""
    try:
        data = json.loads(config_path().read_text())
    except (OSError, json.JSONDecodeError):
        return dict(_DEFAULTS)
    if not isinstance(data, dict):
        return dict(_DEFAULTS)
    return {**_DEFAULTS, **data}


def save(data: dict) -> None:
    """Write the config dict as pretty JSON, creating the file if absent."""
    config_path().write_text(json.dumps(data, indent=2) + "\n")


def get_viewer() -> str:
    """Return the persisted viewer, falling back to the default if it is unknown."""
    viewer = load().get("viewer")
    return viewer if viewer in VIEWERS else _DEFAULTS["viewer"]


def set_viewer(viewer: str) -> None:
    """Persist ``viewer`` into the config, preserving any other keys already there."""
    if viewer not in VIEWERS:
        raise ValueError(f"unknown viewer {viewer!r}; expected one of {VIEWERS}")
    data = load()
    data["viewer"] = viewer
    save(data)
