"""Export the launcher registry as versioned JSON — the contract for out-of-process front ends.

The TUI walks :mod:`fm_tui.registry` in-process to draw its menu. A native front
end (the ``fm-desktop`` macOS app) runs in a different language and process, so it
cannot import the dataclasses. This module serialises the same registry — actions,
their optional modes, robots, variants, backends, and the defaults — to a stable JSON
document those front ends read once and rebuild their menu from.

The document carries everything :meth:`fm_tui.registry.LaunchSpec.command` needs to
rebuild a launch argv without importing Python: each wired action's launch spec
(package, launch file, the arg names, its form ``fields``, the ``viewer_aware`` flag)
plus the viewer options and the standing viewer default. The ``fields`` also let a
front end render the parameter form (camera, rotate_deg, gripper, …) that a fielded
action — e.g. vision teleop — needs. A front end that mirrors ``command()`` against
this JSON produces byte-identical argv — the round-trip test guards that.

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
from fm_tui.registry import Action, Field, LaunchSpec, Mode, PubSpec, actions

# Bump on any breaking shape change to the exported document. Readers pin the
# major they understand and refuse anything newer.
#   2  added the optional per-action `modes` level (action -> mode -> robot -> …).
#   3  added each launch's form `fields` (so a front end can build the parameter
#      form — camera, rotate_deg, gripper, …) and each robot's `variant_labels`.
#   4  added the optional per-action `pub` (a non-launch control action that
#      publishes a one-shot topic — Record toggles /capture/record on the rig).
SCHEMA_VERSION = 4


def _field_to_dict(field_spec: Field) -> dict:
    """Serialise one form field — everything a front end needs to render the form
    step AND rebuild argv the way ``command()`` does (host_only skip, show_if gate,
    default fallback, declaration order). All seven attributes; tuples -> lists."""
    return {
        "name": field_spec.name,
        "label": field_spec.label,
        "default": field_spec.default,
        "required": field_spec.required,
        "choices": list(field_spec.choices),
        "host_only": field_spec.host_only,
        "show_if": list(field_spec.show_if),  # [] = always shown; else [ctrl, value]
    }


def _pub_to_dict(pub: PubSpec) -> dict:
    """Serialise a pub spec — the topic, message type, and labelled payloads a non-launch
    control action publishes. A front end rebuilds the ``ros2 topic pub`` argv from these
    exactly as :meth:`PubSpec.command` does (``--times 3 --rate 5``, ``{data: <value>}``)."""
    return {
        "topic": pub.topic,
        "msg_type": pub.msg_type,
        "options": [[label, value] for label, value in pub.options],
    }


def _launch_to_dict(launch: LaunchSpec) -> dict:
    """Serialise a launch spec — every field ``command()`` reads to build argv."""
    return {
        "package": launch.package,
        "launch_file": launch.launch_file,
        "robot_arg": launch.robot_arg,
        "variant_arg": launch.variant_arg,
        "backend_arg": launch.backend_arg,
        "fields": [_field_to_dict(f) for f in launch.fields],
        "viewer_aware": launch.viewer_aware,
    }


def _robots_to_list(robots) -> list:
    """Serialise a robots tuple — key, label, variants, the default variant, and the
    optional friendly variant labels (raw key -> menu text; the dispatched value stays
    the raw key)."""
    return [
        {
            "key": robot.key,
            "label": robot.label,
            "variants": list(robot.variants),
            "default_variant": robot.default_variant,
            "variant_labels": dict(robot.variant_labels),
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
        "pub": _pub_to_dict(action.pub) if action.pub else None,
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
