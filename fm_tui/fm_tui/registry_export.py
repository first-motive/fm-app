"""Export the launcher registry as versioned JSON — the contract for out-of-process front ends.

The TUI walks :mod:`fm_tui.registry` in-process to draw its menu. A native front
end (the ``fm-desktop`` macOS app) runs in a different language and process, so it
cannot import the dataclasses. This module serialises the same registry — actions,
their optional modes, robots, variants, backends, and the defaults — to a stable JSON
document those front ends read once and rebuild their menu from.

The document carries everything :meth:`fm_tui.registry.LaunchSpec.command` needs to
rebuild a launch argv without importing Python: each wired action's launch spec
(package, launch file, the arg names, the ``viewer_aware`` flag) plus the viewer
options and the standing viewer default. A front end that mirrors ``command()``
against this JSON produces byte-identical argv — the round-trip test guards that.

Invocation (installed as the ``registry`` console script)::

    ros2 run fm_tui registry --json      # pretty JSON to stdout

``SCHEMA_VERSION`` is bumped whenever the shape changes so a reader can refuse a
document it does not understand.
"""

from __future__ import annotations

import argparse
import json
import sys

from fm_tui import config
from fm_tui.registry import Action, LaunchSpec, Mode, actions

# Bump on any breaking shape change to the exported document. Readers pin the
# major they understand and refuse anything newer.
#   2  added the optional per-action `modes` level (action -> mode -> robot -> …).
SCHEMA_VERSION = 2


def _launch_to_dict(launch: LaunchSpec) -> dict:
    """Serialise a launch spec — every field ``command()`` reads to build argv."""
    return {
        "package": launch.package,
        "launch_file": launch.launch_file,
        "robot_arg": launch.robot_arg,
        "variant_arg": launch.variant_arg,
        "backend_arg": launch.backend_arg,
        "viewer_aware": launch.viewer_aware,
    }


def _robots_to_list(robots) -> list:
    """Serialise a robots tuple — key, label, variants, and the default variant."""
    return [
        {
            "key": robot.key,
            "label": robot.label,
            "variants": list(robot.variants),
            "default_variant": robot.default_variant,
        }
        for robot in robots
    ]


def _mode_to_dict(mode: Mode) -> dict:
    """Serialise one mode: the launch-bearing shape (launch, robots, backends) a reader
    treats as the effective action once the operator picks it."""
    return {
        "key": mode.key,
        "label": mode.label,
        "wired": mode.wired,
        "launch": _launch_to_dict(mode.launch) if mode.launch else None,
        "robots": _robots_to_list(mode.robots),
        "backends": list(mode.backends),
    }


def _action_to_dict(action: Action) -> dict:
    """Serialise one action: its launch spec (or null for a stub/mode group), robots,
    backends, and any modes. A mode group carries no launch/robots of its own; its
    launch-bearing shape lives in each entry of ``modes``."""
    return {
        "key": action.key,
        "label": action.label,
        "wired": action.wired,
        "launch": _launch_to_dict(action.launch) if action.launch else None,
        "robots": _robots_to_list(action.robots),
        "backends": list(action.backends),
        "modes": [_mode_to_dict(mode) for mode in action.modes],
    }


def to_dict() -> dict:
    """Build the full registry document: version, viewer defaults, and every action."""
    return {
        "version": SCHEMA_VERSION,
        "viewers": list(config.VIEWERS),
        "default_viewer": config.get_viewer(),
        "actions": [_action_to_dict(action) for action in actions()],
    }


def to_json(*, indent: int = 2) -> str:
    """Render the registry document as JSON with a trailing newline."""
    return json.dumps(to_dict(), indent=indent) + "\n"


def main() -> None:
    """CLI entry point: print the registry as JSON to stdout."""
    parser = argparse.ArgumentParser(
        prog="registry",
        description="Export the fm_tui launch registry as JSON.",
    )
    # --json is the documented, default (and only) format. Accepting it explicitly
    # keeps the `registry --json` call in the docs literal and leaves room for
    # other formats to join behind their own flags later.
    parser.add_argument(
        "--json", action="store_true", help="emit the registry as JSON (default)"
    )
    parser.add_argument(
        "--indent", type=int, default=2, help="JSON indent width (default: 2)"
    )
    args = parser.parse_args()
    sys.stdout.write(to_json(indent=args.indent))


if __name__ == "__main__":
    main()
