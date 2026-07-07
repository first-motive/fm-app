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
    assert {r.key for r in rd.robots} == {"g1_d", "so101", "openarm"}


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
        assert {r.key for r in entry.robots} == {"openarm", "so101", "g1_d"}
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


def test_vision_is_wired_with_fields_and_openarm_only():
    entry = action("vision")
    assert entry.wired
    assert entry.has_backends
    assert entry.launch.has_fields
    assert {r.key for r in entry.robots} == {"openarm"}
    assert entry.robots[0].variants == ("right_arm",)
    assert set(entry.backends) >= {"mujoco", "mock"}
    # Every field name must be a launch arg vision_session.launch.py declares.
    names = [f.name for f in entry.launch.fields]
    assert names == [
        "camera_source",
        "rotate_deg",
        "tracking_mode",
        "publish_debug_image",
        "record",
        "gripper",
    ]


def test_vision_command_appends_fields_after_backend():
    spec = action("vision").launch
    cmd = spec.command(
        "openarm",
        "right_arm",
        "mujoco",
        params={
            "camera_source": "http://host.docker.internal:8090/video",
            "rotate_deg": "90",
            "tracking_mode": "hand",
            "publish_debug_image": "true",
            "record": "true",
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
        "camera_source:=http://host.docker.internal:8090/video",
        "rotate_deg:=90",
        "tracking_mode:=hand",
        "publish_debug_image:=true",
        "record:=true",
        "gripper:=off",
    ]


def test_vision_command_uses_field_defaults_when_params_absent():
    spec = action("vision").launch
    cmd = spec.command("openarm", "right_arm", "mujoco")
    # Fields fall back to their declared defaults, appended in declaration order.
    assert cmd[-6:] == [
        "camera_source:=http://host.docker.internal:8090/video",
        "rotate_deg:=90",
        "tracking_mode:=hand",
        "publish_debug_image:=true",
        "record:=true",
        "gripper:=off",
    ]


def test_fieldless_command_appends_no_extra_args():
    # A spec without fields (simulation) must not gain trailing name:=value tokens.
    cmd = action("simulation").launch.command("openarm", "right_arm", "mujoco")
    assert cmd[-1] == "sim_backend:=mujoco"


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
