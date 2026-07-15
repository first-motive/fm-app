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
            assert got["variant_labels"] == dict(exp.variant_labels)


def test_backends_round_trip():
    src = {a.key: a for a in actions()}
    for action in _roundtrip()["actions"]:
        assert action["backends"] == list(src[action["key"]].backends)


def test_modes_round_trip_with_launch_bearing_shape():
    src = {a.key: a for a in actions()}
    for action in _roundtrip()["actions"]:
        want = src[action["key"]]
        assert [m["key"] for m in action["modes"]] == [m.key for m in want.modes]
        for got, exp in zip(action["modes"], want.modes):
            assert got["label"] == exp.label
            assert got["wired"] == exp.wired
            assert got["backends"] == list(exp.backends)
            assert [r["key"] for r in got["robots"]] == [r.key for r in exp.robots]
            if exp.launch:
                assert got["launch"]["launch_file"] == exp.launch.launch_file


def test_teleoperation_group_serialises_its_modes():
    by_key = {a["key"]: a for a in _roundtrip()["actions"]}
    teleop = by_key["teleoperation"]
    # A mode group carries no launch of its own; its modes hold the launch-bearing shape.
    assert teleop["wired"] is False
    assert teleop["launch"] is None
    assert [m["key"] for m in teleop["modes"]] == ["vision_mirror", "leader_follower"]
    assert all(m["launch"] is not None for m in teleop["modes"])


def _argv_from_launch(launch: dict, robot, variant, backend=None, params=None, viewer=None):
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
    # Fields, between backend and viewer, in declaration order — the same rules
    # command() applies: host_only fields drive a host-side relay (never argv);
    # everything else falls back to its default when the operator gave no value.
    params = params or {}
    for f in launch["fields"]:
        if f["host_only"]:
            continue
        argv.append(f"{f['name']}:={params.get(f['name'], f['default'])}")
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


# --- form fields (schema 3): the vision form the desktop rebuilds ------------------


def _vision_launch_src():
    """The in-process vision-mirror LaunchSpec (source of truth)."""
    teleop = next(a for a in actions() if a.key == "teleoperation")
    return next(m for m in teleop.modes if m.key == "vision_mirror").launch


def _vision_launch_json():
    """The serialised vision-mirror launch dict (as a reader sees it)."""
    teleop = next(a for a in _roundtrip()["actions"] if a["key"] == "teleoperation")
    return next(m for m in teleop["modes"] if m["key"] == "vision_mirror")["launch"]


def test_launch_fields_round_trip():
    src = _vision_launch_src()
    got = _vision_launch_json()["fields"]
    assert src.fields, "vision launch is expected to carry form fields"
    assert len(got) == len(src.fields)
    for g, s in zip(got, src.fields):
        assert g["name"] == s.name
        assert g["label"] == s.label
        assert g["default"] == s.default
        assert g["required"] == s.required
        assert g["choices"] == list(s.choices)
        assert g["host_only"] == s.host_only
        assert g["show_if"] == list(s.show_if)


def test_fieldless_launch_carries_empty_fields():
    # A non-fielded launch (simulation) still serialises `fields` as an empty list,
    # so a reader's field loop is a no-op rather than a missing key.
    sim = {a["key"]: a for a in _roundtrip()["actions"]}["simulation"]["launch"]
    assert sim["fields"] == []


def test_vision_form_carries_host_only_and_show_if():
    fields = {f["name"]: f for f in _vision_launch_json()["fields"]}
    # The camera picker + its conditional IP drive a host-side relay, not argv.
    assert fields["camera"]["host_only"] is True
    assert fields["phone_ip"]["host_only"] is True
    # phone_ip appears only while Camera == "phone".
    assert fields["phone_ip"]["show_if"] == ["camera", "phone"]
    # A normal launch arg is neither host_only nor conditional.
    assert fields["rotate_deg"]["host_only"] is False
    assert fields["rotate_deg"]["show_if"] == []


def test_vision_argv_parity_empty_params():
    # The load-bearing case: no operator input -> host_only fields excluded, every
    # other field falls back to its declared default. Must match command() exactly.
    launch = _vision_launch_json()
    spec = _vision_launch_src()
    assert _argv_from_launch(launch, "openarm", "right_arm", backend="mujoco") == \
        spec.command("openarm", "right_arm", "mujoco")


def test_vision_argv_parity_with_params():
    # Explicit params, including a host_only one that must NOT reach argv.
    launch = _vision_launch_json()
    spec = _vision_launch_src()
    params = {
        "camera": "phone", "phone_ip": "192.168.1.207", "rotate_deg": "90",
        "tracking_mode": "hand", "gripper": "on",
    }
    argv = _argv_from_launch(
        launch, "openarm", "default_bimanual", backend="mujoco", params=params
    )
    assert argv == spec.command("openarm", "default_bimanual", "mujoco", params=params)
    # host_only fields never leak into the launch argv.
    assert not any(a.startswith(("camera:=", "phone_ip:=")) for a in argv)
