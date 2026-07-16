"""Registry tests: structure is valid and the wired/mode-group/stub split holds."""

from fm_tui.registry import ACTIONS, Robot, action, actions


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


def test_autonomous_is_a_stub():
    entry = action("autonomous")
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
    ]
    # Every mode is wired and keeps the backend step.
    for mode in entry.modes:
        assert mode.wired
        assert mode.has_backends


def test_leader_follower_mode_carries_the_teleop_launch():
    lf = _mode("teleoperation", "leader_follower")
    assert lf.launch.launch_file == "teleop.launch.py"
    assert {r.key for r in lf.robots} == {"openarm", "so101", "g1_d", "axol"}
    assert "mujoco" in lf.backends


def test_remote_mirror_mode_targets_the_remote_client_launch():
    rm = _mode("teleoperation", "remote_mirror")
    assert rm.wired
    assert rm.launch.launch_file == "remote_client.launch.py"
    assert rm.has_backends
    # Single-arm only — the rig tracks one hand.
    assert {r.key for r in rm.robots} == {"openarm"}
    assert rm.robots[0].variants == ("right_arm",)
    assert set(rm.backends) >= {"mujoco", "mock"}
    # No form fields — it's rig-sourced (no local camera to configure).
    assert not rm.launch.has_fields


def test_data_capture_is_wired_and_records():
    entry = action("data_capture")
    assert entry.wired
    assert entry.has_backends
    # fm_data is absent in this checkout, so it degrades to the vision session.
    assert entry.launch.launch_file == "vision_session.launch.py"
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
        "camera",
        "phone_ip",
        "rotate_deg",
        "tracking_mode",
        "capture_hands",
        "camera_input",
        "gripper",
    ]
    # The camera picker is host_only: collected + persisted for the host relay
    # manager, never a launch arg. Every other field must match a vision_session arg.
    assert {f.name for f in entry.launch.fields if f.host_only} == {"camera", "phone_ip"}
    # The phone IP is form-conditional: shown only when Camera == "phone".
    phone_ip = next(f for f in entry.launch.fields if f.name == "phone_ip")
    assert phone_ip.show_if == ("camera", "phone")


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
            "camera": "phone",
            "phone_ip": "192.168.1.207",
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
    # host_only picker fields never reach the launch argv (they drive the host relay).
    assert not any(a.startswith(("camera:=", "phone_ip:=")) for a in cmd)
    assert not any("camera_source" in a for a in cmd)


def test_vision_command_uses_field_defaults_when_params_absent():
    spec = _mode("teleoperation", "vision_mirror").launch
    cmd = spec.command("openarm", "right_arm", "mujoco")
    # Only the non-host_only fields fall back to defaults, in declaration order.
    assert cmd[-5:] == [
        "rotate_deg:=90",
        "tracking_mode:=hand",
        "capture_hands:=right",
        "camera_input:=device",
        "gripper:=off",
    ]
    # camera / phone_ip are host_only, so the picker never reaches the argv (camera_input
    # is a separate real launch arg and IS expected).
    assert not any(a.startswith(("camera:=", "phone_ip:=", "camera_source")) for a in cmd)


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
