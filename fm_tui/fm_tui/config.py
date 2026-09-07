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

# The transports a host can speak. Mirrors the identity card's `transport` enum
# and fm-comms' profile names, because these three must agree — a value the TUI
# offers that the card refuses is a setting the operator cannot actually apply.
#   zenoh     DDS on loopback, one bridge per host, one router. The default.
#   dds-lan   FastDDS pinned to the LAN interface. The labelled escape hatch.
TRANSPORTS = ("zenoh", "dds-lan")

# Where a requested transport is parked until run.sh can apply it.
#
# The TUI does NOT write the identity card, and must not. The card is a fact
# about the machine, living at /etc/fm/machine.json (or ~/.config on macOS), and
# the launcher usually runs inside a container where that path is not mounted and
# where writing it would mean writing the container's own filesystem — a setting
# that vanishes on teardown while appearing to have been saved.
#
# So the launcher records a request here, in the config file that IS on the
# mounted host repo root, and `./run.sh` applies it on the next launch through
# the same single writer the `--comms` flag uses. One writer for the card, and a
# TUI that can still change the setting from where it actually runs.
_PENDING_TRANSPORT_KEY = "comms"

# The viewers the launcher can dispatch. get_viewer() falls back to the default
# for any value outside this set, so a hand-edited config can never wedge the UI.
#   foxglove  the Foxglove desktop app over foxglove_bridge (:8765)
#   rviz      rviz2 (native on Linux; browser-over-VNC on macOS)
#   panel     the fm_viewer browser page over the same foxglove_bridge (:8765) —
#             keeps the bridge up but opens no Foxglove desktop app
VIEWERS = ("foxglove", "rviz", "panel")


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


def active_transport() -> str:
    """Return the transport this process is actually running on.

    Read from the environment rather than the config file: ``run.sh`` resolves the
    host's transport from its identity card and exports ``FM_COMMS_PROFILE``, and
    that resolved answer — not a preference written down somewhere — is what the
    launch will speak. Absent (an ad-hoc run outside run.sh), the fleet default.
    """
    profile = os.environ.get("FM_COMMS_PROFILE", "")
    return profile if profile in TRANSPORTS else TRANSPORTS[0]


def get_pending_transport() -> str | None:
    """Return the transport requested but not yet applied, or ``None``."""
    value = load().get(_PENDING_TRANSPORT_KEY)
    return value if value in TRANSPORTS else None


def set_pending_transport(transport: str) -> None:
    """Record a transport for ``run.sh`` to apply on the next launch.

    Requesting the transport already in force clears the request instead of
    recording a no-op, so toggling away and back leaves nothing behind for
    run.sh to apply.
    """
    if transport not in TRANSPORTS:
        raise ValueError(f"unknown transport {transport!r}; expected one of {TRANSPORTS}")
    data = load()
    if transport == active_transport():
        data.pop(_PENDING_TRANSPORT_KEY, None)
    else:
        data[_PENDING_TRANSPORT_KEY] = transport
    save(data)
