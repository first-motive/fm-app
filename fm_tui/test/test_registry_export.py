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
    assert [m["key"] for m in teleop["modes"]] == [
        "vision_mirror",
        "remote_mirror",
        "leader_follower",
    ]
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


def test_vision_form_has_no_host_only_or_conditional_fields():
    # The phone/webcam picker (the only host_only + show_if fields) was removed; every
    # field is now a plain vision_session launch arg. The plumbing still serialises.
    fields = _vision_launch_json()["fields"]
    assert fields, "vision launch is expected to carry form fields"
    assert not any(f["host_only"] for f in fields)
    assert all(f["show_if"] == [] for f in fields)


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
        "rotate_deg": "90", "tracking_mode": "hand", "gripper": "on",
    }
    argv = _argv_from_launch(
        launch, "openarm", "default_bimanual", backend="mujoco", params=params
    )
    assert argv == spec.command("openarm", "default_bimanual", "mujoco", params=params)


# --- pub actions (schema 4): the control action the desktop rebuilds ----------------


def _pub_action_src():
    """The in-process Record action (source of truth for its PubSpec)."""
    return next(a for a in actions() if a.key == "record")


def _pub_action_json():
    """The serialised Record action (as a reader sees it)."""
    return {a["key"]: a for a in _roundtrip()["actions"]}["record"]


def test_pub_action_round_trips():
    src = _pub_action_src()
    got = _pub_action_json()
    assert src.is_pub and src.pub is not None
    # A pub action carries no launch — a reader dispatches it via `pub`, not a launch argv.
    assert got["launch"] is None
    assert got["wired"] is False
    assert got["pub"]["topic"] == src.pub.topic
    assert got["pub"]["msg_type"] == src.pub.msg_type
    assert got["pub"]["options"] == [list(o) for o in src.pub.options]


def test_launch_actions_carry_null_pub():
    # A launch/mode/stub action serialises `pub` as null, so a reader's branch is
    # unambiguous rather than a missing key.
    by_key = {a["key"]: a for a in _roundtrip()["actions"]}
    assert by_key["simulation"]["pub"] is None
    assert by_key["teleoperation"]["pub"] is None
    assert by_key["autonomous"]["pub"] is None


def _argv_from_pub(pub: dict, value: str):
    """Rebuild the one-shot ``ros2 topic pub`` argv from the serialised pub dict — the
    exact logic an out-of-process front end reimplements (mirrors ``PubSpec.command``)."""
    return [
        "ros2", "topic", "pub", "--times", "3", "--rate", "5",
        pub["topic"], pub["msg_type"], "{data: %s}" % value,
    ]


def test_pub_argv_rebuilt_from_json_matches_command():
    src = _pub_action_src().pub
    got = _pub_action_json()["pub"]
    for _label, value in src.options:
        assert _argv_from_pub(got, value) == src.command(value)
