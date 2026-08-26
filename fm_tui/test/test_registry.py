"""Registry tests: structure is valid and the wired/mode-group/stub split holds."""

from fm_tui.registry import ACTIONS, Robot, _has_package, action, actions


def _mode(action_key, mode_key):
    """Look up a mode by key under a mode-grouping action."""
    for mode in action(action_key).modes:
        if mode.key == mode_key:
            return mode
    raise KeyError(f"{action_key}/{mode_key}")


def test_actions_have_unique_keys():
    keys = [a.key for a in actions()]
    assert len(keys) == len(set(keys))
    assert keys == [a.key for a in ACTIONS]


def test_robot_description_is_wired_with_robots():
    rd = action("robot_description")
    assert rd.wired
    assert rd.launch is not None
    assert rd.robots
    assert {r.key for r in rd.robots} == {"g1_d", "so101", "openarm", "axol"}


def test_autonomous_follows_the_policy_layer():
    # The autonomous launch lives in fm_policy_serve, which is private and absent from a
    # public checkout: the action is wired exactly where that package is installed.
    entry = action("autonomous")
    if _has_package("fm_policy_serve"):
        assert entry.wired
        assert entry.launch.launch_file == "autonomous.launch.py"
        assert entry.launch.package == "fm_policy_serve"
        assert [f.name for f in entry.launch.fields] == ["policy_type", "checkpoint"]
        assert entry.robots
    else:
        assert not entry.wired
        assert entry.launch is None
        assert entry.robots == ()


def test_simulation_is_wired_with_backends():
    entry = action("simulation")
    assert entry.wired
    assert entry.has_backends
    assert not entry.has_modes
    assert {r.key for r in entry.robots} == {"openarm", "so101", "g1_d", "axol"}
    assert "mujoco" in entry.backends


def test_teleoperation_is_a_mode_group():
    entry = action("teleoperation")
    # A mode group carries no launch/robots of its own — it dispatches through its modes.
    assert entry.has_modes
    assert not entry.wired
    assert entry.launch is None
    assert entry.robots == ()
    assert [m.key for m in entry.modes] == [
        "vision_mirror",
        "remote_mirror",
        "leader_follower",
        "vr",
    ]
    # Every mode is wired and keeps the backend step.
    for mode in entry.modes:
        assert mode.wired
        assert mode.has_backends


def test_leader_follower_mode_carries_its_session_launch():
    # A mode cannot pass a fixed launch argument, so the input lives in the launch file.
    lf = _mode("teleoperation", "leader_follower")
    assert lf.launch.launch_file == "leader_session.launch.py"
    assert {r.key for r in lf.robots} == {"openarm", "so101", "g1_d", "axol"}
    assert "mujoco" in lf.backends


def test_vr_mode_carries_its_session_launch():
    vr = _mode("teleoperation", "vr")
    assert vr.launch.launch_file == "vr_session.launch.py"
    assert {r.key for r in vr.robots} == {"openarm", "so101", "g1_d", "axol"}
    assert "mujoco" in vr.backends


def test_remote_mirror_mode_targets_the_remote_client_launch():
    rm = _mode("teleoperation", "remote_mirror")
    assert rm.wired
    assert rm.launch.launch_file == "remote_client.launch.py"
    assert rm.has_backends
    # right_arm mirrors the one control hand; default_bimanual mirrors both hands.
    assert {r.key for r in rm.robots} == {"openarm"}
    assert rm.robots[0].variants == ("right_arm", "default_bimanual")
    assert rm.robots[0].default_variant == "right_arm"
    assert set(rm.backends) >= {"mujoco", "mock"}
    # Rig-sourced (no local camera), but the gripper toggle rides as a form field.
    assert [f.name for f in rm.launch.fields] == ["gripper"]


def test_data_capture_is_wired_and_records():
    entry = action("data_capture")
    assert entry.wired
    assert entry.has_backends
    expected_launch = (
        "record.launch.py" if _has_package("fm_data") else "vision_session.launch.py"
    )
    assert entry.launch.launch_file == expected_launch
    # record:=true rides in the argv (the record field defaults "true").
    cmd = entry.launch.command("openarm", "right_arm", "mujoco")
    assert "record:=true" in cmd


def test_record_is_a_pub_action():
    entry = action("record")
    # A control action: no launch, no robots — it publishes a topic to the rig recorder.
    assert entry.is_pub
    assert not entry.wired
    assert entry.launch is None
    assert entry.robots == ()
    assert entry.pub.topic == "/capture/record"
    assert entry.pub.msg_type == "std_msgs/msg/Bool"
    assert entry.pub.options == (("Start", "true"), ("Stop", "false"))


def test_pub_command_builds_topic_pub_argv():
    pub = action("record").pub
    # --times 3 --rate 5 (idempotent Bool) rather than --once, so a discovery-race
    # drop still lands without double-triggering a take.
    assert pub.command("true") == [
        "ros2", "topic", "pub", "--times", "3", "--rate", "5",
        "/capture/record", "std_msgs/msg/Bool", "{data: true}",
    ]
    assert pub.command("false")[-1] == "{data: false}"


def test_every_robot_default_is_a_listed_variant():
    def robots_of(entry):
        yield from entry.robots
        for mode in getattr(entry, "modes", ()):
            yield from mode.robots

    for entry in actions():
        for robot in robots_of(entry):
            assert robot.default_variant in robot.variants


def test_launch_command_wires_robot_and_variant():
    spec = action("robot_description").launch
    cmd = spec.command("openarm", "left_arm")
    assert cmd == [
        "ros2",
        "launch",
        "fm_description",
        "view_robot.launch.py",
        "robot:=openarm",
        "variant:=left_arm",
    ]


def test_launch_command_appends_backend_when_set():
    spec = action("simulation").launch
    cmd = spec.command("openarm", "right_arm", "mujoco")
    assert cmd[-3:] == ["robot:=openarm", "variant:=right_arm", "sim_backend:=mujoco"]
    # No backend arg -> no trailing sim_backend.
    assert "sim_backend:=mujoco" not in action("robot_description").launch.command(
        "openarm", "right_arm", "mujoco"
    )


def test_vision_is_wired_with_fields_and_openarm_only():
    entry = _mode("teleoperation", "vision_mirror")
    assert entry.wired
    assert entry.has_backends
    assert entry.launch.has_fields
    assert {r.key for r in entry.robots} == {"openarm"}
    assert entry.robots[0].variants == ("right_arm", "default_bimanual")
    assert set(entry.backends) >= {"mujoco", "mock"}
    names = [f.name for f in entry.launch.fields]
    assert names == [
        "rotate_deg",
        "tracking_mode",
        "capture_hands",
        "camera_input",
        "gripper",
    ]
    # The phone/webcam picker was removed — every field now maps to a vision_session
    # launch arg, so none are host_only or form-conditional.
    assert not any(f.host_only for f in entry.launch.fields)
    assert not any(f.show_if for f in entry.launch.fields)


def test_vision_openarm_variant_labels():
    # The bimanual option is surfaced with a friendly menu label; the dispatched
    # variant value stays the raw key (asserted in test_launcher).
    robot = _mode("teleoperation", "vision_mirror").robots[0]
    assert robot.variant_labels == {
        "right_arm": "Right arm only",
        "default_bimanual": "Both arms (bimanual)",
    }


def test_vision_command_appends_only_launch_fields_after_backend():
    spec = _mode("teleoperation", "vision_mirror").launch
    cmd = spec.command(
        "openarm",
        "right_arm",
        "mujoco",
        params={
            "rotate_deg": "90",
            "tracking_mode": "hand",
            "capture_hands": "both",
            "camera_input": "topic",
            "gripper": "off",
        },
    )
    assert cmd == [
        "ros2",
        "launch",
        "fm_bringup",
        "vision_session.launch.py",
        "robot:=openarm",
        "variant:=right_arm",
        "sim_backend:=mujoco",
        "rotate_deg:=90",
        "tracking_mode:=hand",
        "capture_hands:=both",
        "camera_input:=topic",
        "gripper:=off",
    ]
    # camera_source is a vision_session launch default, not a TUI field — never in the argv.
    assert not any("camera_source" in a for a in cmd)


def test_vision_command_uses_field_defaults_when_params_absent():
    spec = _mode("teleoperation", "vision_mirror").launch
    cmd = spec.command("openarm", "right_arm", "mujoco")
    # All fields fall back to their defaults, in declaration order.
    assert cmd[-5:] == [
        "rotate_deg:=90",
        "tracking_mode:=hand",
        "capture_hands:=right",
        "camera_input:=device",
        "gripper:=off",
    ]
    # camera_source is a vision_session launch default, not a TUI field — never in the argv.
    assert not any(a.startswith("camera_source") for a in cmd)


def test_fieldless_command_appends_no_extra_args():
    # A spec without fields (simulation) must not gain trailing name:=value tokens.
    cmd = action("simulation").launch.command("openarm", "right_arm", "mujoco")
    assert cmd[-1] == "sim_backend:=mujoco"


def test_viewer_aware_command_appends_viewer_flags():
    spec = action("robot_description").launch
    assert spec.viewer_aware
    assert spec.command("g1_d", "g1_d", viewer="rviz")[-2:] == [
        "use_foxglove:=false",
        "use_rviz:=true",
    ]
    assert spec.command("g1_d", "g1_d", viewer="foxglove")[-2:] == [
        "use_foxglove:=true",
        "use_rviz:=false",
    ]
    # Joint control is derived from use_rviz in the launch, so no jsp flag rides
    # in the argv (keeps parity with FM Desktop's identical command builder).
    assert not any("use_jsp_gui" in a for a in spec.command("g1_d", "g1_d", viewer="rviz"))
    # No viewer passed -> no viewer flags (the launch file's own defaults win).
    assert "use_rviz:=false" not in spec.command("g1_d", "g1_d")


def test_non_viewer_aware_command_ignores_viewer():
    spec = action("simulation").launch
    assert not spec.viewer_aware
    cmd = spec.command("g1_d", "g1_d", "mujoco", viewer="rviz")
    assert not any(arg.startswith("use_rviz") for arg in cmd)
    assert not any(arg.startswith("use_foxglove") for arg in cmd)


def test_robot_rejects_default_outside_variants():
    try:
        Robot(key="x", label="X", variants=("a",), default_variant="b")
    except ValueError:
        return
    raise AssertionError("expected ValueError for default outside variants")


def test_robot_rejects_variant_label_for_unknown_variant():
    try:
        Robot(
            key="x", label="X", variants=("a",), default_variant="a",
            variant_labels={"nope": "Nope"},
        )
    except ValueError:
        return
    raise AssertionError("expected ValueError for variant_labels outside variants")


def test_action_lookup_missing_raises():
    try:
        action("does_not_exist")
    except KeyError:
        return
    raise AssertionError("expected KeyError for unknown action")
