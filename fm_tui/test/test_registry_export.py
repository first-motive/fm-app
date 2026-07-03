"""Registry-export tests: the JSON round-trips the registry and carries enough for argv.

Two guarantees an out-of-process front end depends on:

- **Round-trip** — every action, robot, variant, backend, and default in
  :mod:`fm_tui.registry` survives the JSON dump unchanged.
- **Argv parity** — a front end that rebuilds a launch argv from the JSON (this
  test mirrors ``LaunchSpec.command`` against the serialised launch dict) produces
  the same argv the in-process registry does.
"""

import json

from fm_tui import config
from fm_tui.registry import actions
from fm_tui.registry_export import SCHEMA_VERSION, to_dict, to_json


def _roundtrip() -> dict:
    """The document as a reader sees it: dumped to JSON text and parsed back."""
    return json.loads(to_json())


def test_document_carries_version_and_viewer_defaults():
    doc = _roundtrip()
    assert doc["version"] == SCHEMA_VERSION
    assert doc["viewers"] == list(config.VIEWERS)
    assert doc["default_viewer"] in config.VIEWERS


def test_actions_round_trip_keys_and_labels_in_order():
    doc = _roundtrip()
    assert [a["key"] for a in doc["actions"]] == [a.key for a in actions()]
    assert [a["label"] for a in doc["actions"]] == [a.label for a in actions()]


def test_wired_and_stub_split_survives():
    by_key = {a["key"]: a for a in _roundtrip()["actions"]}
    assert by_key["robot_description"]["wired"] is True
    assert by_key["robot_description"]["launch"] is not None
    assert by_key["autonomous"]["wired"] is False
    assert by_key["autonomous"]["launch"] is None
    assert by_key["autonomous"]["robots"] == []


def test_robots_variants_and_defaults_round_trip():
    src = {a.key: a for a in actions()}
    for action in _roundtrip()["actions"]:
        want = src[action["key"]]
        assert len(action["robots"]) == len(want.robots)
        for got, exp in zip(action["robots"], want.robots):
            assert got["key"] == exp.key
            assert got["label"] == exp.label
            assert got["variants"] == list(exp.variants)
            assert got["default_variant"] == exp.default_variant
            assert got["default_variant"] in got["variants"]


def test_backends_round_trip():
    src = {a.key: a for a in actions()}
    for action in _roundtrip()["actions"]:
        assert action["backends"] == list(src[action["key"]].backends)


def _argv_from_launch(launch: dict, robot, variant, backend=None, viewer=None):
    """Rebuild a launch argv from the serialised launch dict.

    Mirrors ``LaunchSpec.command`` using only JSON fields — the exact logic an
    out-of-process front end reimplements. Parity with ``command()`` is the test.
    """
    argv = [
        "ros2",
        "launch",
        launch["package"],
        launch["launch_file"],
        f"{launch['robot_arg']}:={robot}",
        f"{launch['variant_arg']}:={variant}",
    ]
    if launch["backend_arg"] and backend:
        argv.append(f"{launch['backend_arg']}:={backend}")
    if launch["viewer_aware"] and viewer:
        use_rviz = viewer == "rviz"
        argv.append(f"use_foxglove:={'false' if use_rviz else 'true'}")
        argv.append(f"use_rviz:={'true' if use_rviz else 'false'}")
    return argv


def test_argv_rebuilt_from_json_matches_command():
    src = {a.key: a for a in actions()}
    by_key = {a["key"]: a for a in _roundtrip()["actions"]}

    # viewer-aware action, both viewers
    rd = by_key["robot_description"]["launch"]
    spec = src["robot_description"].launch
    for viewer in ("foxglove", "rviz"):
        assert _argv_from_launch(rd, "g1_d", "g1_d", viewer=viewer) == spec.command(
            "g1_d", "g1_d", viewer=viewer
        )

    # backend action ignores viewer (not viewer-aware), honours backend
    sim = by_key["simulation"]["launch"]
    sim_spec = src["simulation"].launch
    assert _argv_from_launch(
        sim, "openarm", "right_arm", backend="mujoco", viewer="rviz"
    ) == sim_spec.command("openarm", "right_arm", "mujoco", viewer="rviz")


def test_document_is_pure_json_serialisable():
    # No dataclasses or tuples leak through — a strict re-dump must not raise.
    json.dumps(to_dict())
