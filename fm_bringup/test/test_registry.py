"""Registry contract: lookup, the OpenArm entry, and path construction.

Build/SRDF processing (build_description, semantic) needs the full vendored
description + MoveIt config on the share path, so it is exercised by the
container build/smoke runs rather than here. These tests pin the data contract
the launch files depend on.
"""

import pytest

from fm_bringup import registry


def test_get_openarm():
    spec = registry.get("openarm")
    assert spec.key == "openarm"
    assert spec.default_variant == "right_arm"


def test_get_unknown_raises_with_registered_list():
    with pytest.raises(RuntimeError) as exc:
        registry.get("nope")
    assert "openarm" in str(exc.value)


def test_openarm_controller_set():
    spec = registry.get("openarm")
    assert set(spec.controllers) == {"right_arm", "default_bimanual"}
    assert spec.controllers["right_arm"]["active"] == ["openarm_right_arm_controller"]
    assert spec.controllers["right_arm"]["inactive"] == [
        "openarm_right_forward_position_controller"
    ]


def test_openarm_standalone_cm_backends():
    spec = registry.get("openarm")
    assert spec.standalone_cm_backends == frozenset({"mock", "real"})


def test_controllers_file_path_per_variant():
    spec = registry.get("openarm")
    path = spec.controllers_file("right_arm")
    assert path.endswith("config/openarm/right_arm.controllers.yaml")


def test_srdf_selection():
    spec = registry.get("openarm")
    # right_arm is served in-repo; other variants fall back to the MoveIt config.
    assert "right_arm" in spec.bringup_srdf
    assert "default_bimanual" not in spec.bringup_srdf
    assert spec.moveit_srdf == "openarm_bimanual.srdf"


def test_get_so101():
    spec = registry.get("so101")
    assert spec.key == "so101"
    assert spec.default_variant == "so101"
    # Single-config robot: no preset arg, so the description build passes no preset.
    assert spec.preset_arg is None


def test_so101_controller_set():
    spec = registry.get("so101")
    assert set(spec.controllers) == {"so101"}
    assert spec.controllers["so101"]["active"] == [
        "so101_arm_controller",
        "so101_gripper_controller",
    ]


def test_so101_moveit_config_in_repo():
    spec = registry.get("so101")
    # SO101 MoveIt config is authored in fm_bringup, not a vendored package.
    assert spec.moveit_pkg == "fm_bringup"
    assert spec.semantic("so101")  # resolves the in-repo SRDF without error


def test_get_g1_d():
    spec = registry.get("g1_d")
    assert spec.key == "g1_d"
    assert spec.default_variant == "g1_d"
    assert spec.preset_arg is None


def test_g1_d_controller_set():
    spec = registry.get("g1_d")
    assert spec.controllers["g1_d"]["active"] == [
        "g1_right_arm_controller",
        "g1_left_arm_controller",
        "g1_base_controller",
        "g1_right_hand_controller",
        "g1_left_hand_controller",
    ]


def test_g1_d_cmd_vel_remap():
    # diff_drive's cmd_vel_unstamped is remapped to the canonical /cmd_vel.
    spec = registry.get("g1_d")
    assert spec.cmd_remaps == (
        ("/g1_base_controller/cmd_vel_unstamped", "/cmd_vel"),
    )


def test_single_arm_robots_have_no_cmd_remaps():
    assert registry.get("openarm").cmd_remaps == ()
    assert registry.get("so101").cmd_remaps == ()


def test_g1_d_teleop_nodes_include_hand_teleop():
    spec = registry.get("g1_d")
    assert spec.teleop_nodes == (("fm_teleop_device", "g1_hand_teleop"),)


def test_single_arm_robots_have_no_teleop_nodes():
    assert registry.get("openarm").teleop_nodes == ()
    assert registry.get("so101").teleop_nodes == ()


def test_g1_d_servo_nodes_one_per_arm():
    # Right arm on the primary servo_node, left arm on servo_node_left.
    spec = registry.get("g1_d")
    names = [name for name, _ in spec.servo_nodes()]
    assert names == ["servo_node", "servo_node_left"]
    assert spec.servo_nodes()[1][1].endswith("config/g1_d/servo_left.yaml")


def test_single_arm_robots_have_one_servo_node():
    # Robots without extra_servo_configs run just the primary servo_node.
    assert [n for n, _ in registry.get("openarm").servo_nodes()] == ["servo_node"]
    assert [n for n, _ in registry.get("so101").servo_nodes()] == ["servo_node"]


def test_g1_d_real_is_not_a_cm_backend():
    # The G1 real path is the arm_sdk bridge, not a controller_manager.
    spec = registry.get("g1_d")
    assert spec.standalone_cm_backends == frozenset({"mock"})
    assert "real" not in spec.standalone_cm_backends


def test_g1_d_moveit_config_in_repo():
    spec = registry.get("g1_d")
    assert spec.moveit_pkg == "fm_bringup"
    assert spec.semantic("g1_d")  # resolves the in-repo SRDF without error


def test_full_state_jsp_only_for_subset_controlled_g1():
    # The G1-D drives 7 of 34 joints, so it needs the joint_state_publisher; the
    # OpenArm + SO101 control their whole model and must not.
    assert registry.get("g1_d").full_state_jsp is True
    assert registry.get("openarm").full_state_jsp is False
    assert registry.get("so101").full_state_jsp is False


def test_registered_robots():
    assert {"openarm", "so101", "g1_d"} <= set(registry._ROBOTS)
