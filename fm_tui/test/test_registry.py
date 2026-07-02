"""Registry tests: structure is valid and the wired/stub split holds."""

from fm_tui.registry import ACTIONS, Robot, action, actions


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


def test_simulation_and_teleop_are_wired_with_backends():
    for key in ("simulation", "teleop"):
        entry = action(key)
        assert entry.wired
        assert entry.has_backends
        assert {r.key for r in entry.robots} == {"openarm", "so101", "g1_d", "axol"}
        assert "mujoco" in entry.backends


def test_every_robot_default_is_a_listed_variant():
    for entry in actions():
        for robot in entry.robots:
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


def test_action_lookup_missing_raises():
    try:
        action("does_not_exist")
    except KeyError:
        return
    raise AssertionError("expected KeyError for unknown action")
