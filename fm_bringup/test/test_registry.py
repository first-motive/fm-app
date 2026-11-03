"""Registry contract: lookup, the OpenArm entry, and path construction.

Build/SRDF processing (build_description, semantic) needs the full vendored
description + MoveIt config on the share path, so it is exercised by the
container build/smoke runs rather than here. The one exception is the Panda
name cross-check, which builds the URDF and skips when franka_description is
absent. These tests pin the data contract the launch files depend on.
"""

import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from fm_bringup import registry


def _load_yaml(path):
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


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
    assert set(spec.controllers) == {
        "right_arm",
        "right_arm_with_pinch_gripper",
        "default_bimanual",
    }
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


def test_arm_controllers_reads_the_yaml_the_controller_manager_loads():
    # The leader bypass streams to an arm controller, so its topic and joint order come
    # from this one file — never restated in a launch argument.
    arms = registry.get("so101").arm_controllers("so101")
    assert arms == [
        (
            "so101_arm_controller",
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
        )
    ]


def test_arm_controllers_excludes_grippers_and_returns_one_entry_per_arm():
    arms = registry.get("openarm").arm_controllers("default_bimanual")
    assert [name for name, _ in arms] == [
        "openarm_left_arm_controller",
        "openarm_right_arm_controller",
    ]
    assert all(joints for _, joints in arms)


def test_srdf_selection():
    spec = registry.get("openarm")
    # right_arm is served in-repo; other variants fall back to the MoveIt config.
    assert "right_arm" in spec.bringup_srdf
    assert "default_bimanual" not in spec.bringup_srdf
    assert spec.moveit_srdf == "openarm_bimanual.srdf"


def test_openarm_mirror_arms_left_geometry():
    # The left arm's base frame is the right mirrored about the sagittal plane, so it
    # carries its own workspace box (right Y range [-0.45, 0.30] -> [-0.30, 0.45]); the
    # right inherits the vision.yaml defaults and defines no per-arm geometry.
    right, left = registry.get("openarm").mirror_arms["default_bimanual"]
    assert (right["ns"], left["ns"]) == ("right", "left")
    assert left["workspace_min"][1] == -0.30
    assert left["workspace_max"][1] == 0.45
    assert left["axis_map_linear"] == ["z", "x", "-y"]
    assert "workspace_min" not in right
    assert "workspace_max" not in right


def test_openarm_mirror_arms_grippers():
    # Each hand drives its own arm's pinchers; the left mirrors the right prefix with an
    # identical finger range (the reflect is on Y, the finger axis is X).
    right, left = registry.get("openarm").mirror_arms["default_bimanual"]
    assert right["gripper"] == {
        "preset_topic": "/gripper_teleop/right/preset",
        "command_topic": "/openarm_right_gripper_controller/joint_trajectory",
        "joints": ["openarm_right_finger_joint1"],
        "open": [-0.7854], "close": [0.0],
    }
    assert left["gripper"] == {
        "preset_topic": "/gripper_teleop/left/preset",
        "command_topic": "/openarm_left_gripper_controller/joint_trajectory",
        "joints": ["openarm_left_finger_joint1"],
        "open": [-0.7854], "close": [0.0],
    }
    # The preset the mirror_source publishes must match the gripper adapter's subscription.
    assert right["gripper_preset"] == right["gripper"]["preset_topic"]
    assert left["gripper_preset"] == left["gripper"]["preset_topic"]


def test_openarm_mirror_arms_reset_home_ee():
    # Reset drives each arm to its own home EE (the "hands forward" ready pose); the left
    # position is the right with Y negated.
    right, left = registry.get("openarm").mirror_arms["default_bimanual"]
    assert right["reset_home_ee"]["position"] == [0.3399, -0.069, -0.12]
    assert left["reset_home_ee"]["position"] == [0.3399, 0.069, -0.12]
    assert left["command_frame"] == "openarm_left_base_link"
    # 1:1 gain so hand spacing maps to gripper spacing.
    assert right["mirror_gain"] == 1.0 and left["mirror_gain"] == 1.0


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
    # OpenArm + SO101 + Axol control their whole model and must not.
    assert registry.get("g1_d").full_state_jsp is True
    assert registry.get("openarm").full_state_jsp is False
    assert registry.get("so101").full_state_jsp is False
    assert registry.get("axol").full_state_jsp is False


def test_get_axol():
    spec = registry.get("axol")
    assert spec.key == "axol"
    assert spec.default_variant == "axol"
    # Single fixed bimanual configuration: no preset arg.
    assert spec.preset_arg is None
    # Arms only — no diff-drive base or hands, so no remaps or teleop adapters.
    assert spec.cmd_remaps == ()
    assert spec.teleop_nodes == ()


def test_axol_controller_set():
    spec = registry.get("axol")
    assert set(spec.controllers) == {"axol"}
    assert spec.controllers["axol"]["active"] == [
        "axol_right_arm_controller",
        "axol_left_arm_controller",
    ]


def test_axol_servo_nodes_one_per_arm():
    # Right arm on the primary servo_node, left arm on servo_node_left (bimanual).
    spec = registry.get("axol")
    names = [name for name, _ in spec.servo_nodes()]
    assert names == ["servo_node", "servo_node_left"]
    assert spec.servo_nodes()[1][1].endswith("config/axol/servo_left.yaml")


def test_axol_real_is_not_a_cm_backend():
    # Axol's CAN backend is deferred; only mock uses a standalone controller_manager.
    spec = registry.get("axol")
    assert spec.standalone_cm_backends == frozenset({"mock"})
    assert "real" not in spec.standalone_cm_backends


def test_axol_moveit_config_in_repo():
    spec = registry.get("axol")
    assert spec.moveit_pkg == "fm_bringup"
    assert spec.semantic("axol")  # resolves the in-repo SRDF without error


def test_registered_robots():
    assert {"openarm", "so101", "g1_d", "axol", "panda"} <= set(registry._ROBOTS)


def test_get_panda():
    spec = registry.get("panda")
    assert spec.key == "panda"
    assert spec.default_variant == "panda"
    # Single fixed arm + hand configuration: no preset arg.
    assert spec.preset_arg is None
    # Arm + hand only — no base or hand adapter, so no remaps or teleop nodes.
    assert spec.cmd_remaps == ()
    assert spec.teleop_nodes == ()
    assert [n for n, _ in spec.servo_nodes()] == ["servo_node"]


def test_panda_controller_set():
    # Names match fm-sim's panda catalog; the forward-position arm controller loads
    # inactive, like the OpenArm's.
    spec = registry.get("panda")
    assert set(spec.controllers) == {"panda"}
    assert spec.controllers["panda"]["active"] == [
        "panda_arm_controller",
        "panda_gripper_controller",
    ]
    assert spec.controllers["panda"]["inactive"] == [
        "panda_arm_forward_position_controller"
    ]
    # Every spawned controller is defined in the file the controller_manager loads.
    defined = _load_yaml(spec.controllers_file("panda"))["controller_manager"][
        "ros__parameters"
    ]
    cset = spec.controllers["panda"]
    assert set(cset["active"] + cset["inactive"]) <= set(defined)


def test_panda_arm_controller_drives_the_fer_joints():
    # Upstream names the Panda `fer`; the gripper JTC is excluded from the arm set.
    arms = registry.get("panda").arm_controllers("panda")
    assert arms == [
        ("panda_arm_controller", [f"fer_joint{i}" for i in range(1, 8)])
    ]


def test_panda_real_is_not_a_cm_backend():
    # Simulation-only robot: only mock uses a standalone controller_manager.
    spec = registry.get("panda")
    assert spec.standalone_cm_backends == frozenset({"mock"})


def test_panda_mesh_rewrite_points_dae_visuals_at_fm_description_stl():
    pattern, repl = registry.get("panda").mesh_rewrite
    visual = 'filename="package://franka_description/meshes/robots/fer/visual/link0.dae"'
    assert pattern.sub(repl, visual) == (
        'filename="package://fm_description/panda_meshes/robots/fer/visual/link0.stl"'
    )
    collision = 'filename="package://franka_description/meshes/robots/fer/collision/link0.stl"'
    assert pattern.sub(repl, collision) == collision


def test_panda_moveit_config_in_repo():
    spec = registry.get("panda")
    assert spec.moveit_pkg == "fm_bringup"
    srdf = spec.semantic("panda")  # resolves the in-repo SRDF without error
    # Robot name must match the description's <robot name="panda">.
    assert '<robot name="panda">' in srdf
    assert spec.full_state_jsp is False


def test_panda_configs_match_the_urdf():
    # The panda configs are hand-renamed from the MoveIt panda (panda_*) to
    # franka_description's fer_* names; cross-check every name against the built model.
    try:
        get_package_share_directory("franka_description")
    except PackageNotFoundError:
        pytest.skip("franka_description not installed")
    spec = registry.get("panda")
    urdf = ET.fromstring(spec.build_description("panda", "mock"))
    links = {link.get("name") for link in urdf.findall("link")}
    joints = {joint.get("name") for joint in urdf.findall("joint")}

    srdf = ET.fromstring(spec.semantic("panda"))
    assert srdf.get("name") == urdf.get("name") == "panda"
    srdf_links, srdf_joints = set(), set()
    for group in srdf.findall("group"):
        srdf_links |= {e.get("name") for e in group.findall("link")}
        srdf_joints |= {e.get("name") for e in group.findall("joint")}
        for chain in group.findall("chain"):
            srdf_links |= {chain.get("base_link"), chain.get("tip_link")}
    for state in srdf.findall("group_state"):
        srdf_joints |= {e.get("name") for e in state.findall("joint")}
    for pair in srdf.findall("disable_collisions"):
        srdf_links |= {pair.get("link1"), pair.get("link2")}
    srdf_links |= {e.get("parent_link") for e in srdf.findall("end_effector")}
    assert srdf_links - links == set()
    assert srdf_joints - joints == set()

    controllers = _load_yaml(spec.controllers_file("panda"))
    for name, params in controllers.items():
        if name == "controller_manager":
            continue
        assert set(params["ros__parameters"]["joints"]) <= joints, name
    limits = _load_yaml(spec.moveit_file("joint_limits.yaml"))["joint_limits"]
    assert set(limits) <= joints

    servo = _load_yaml(spec.servo_params_file())["moveit_servo"]
    for key in ("planning_frame", "robot_link_command_frame", "ee_frame_name"):
        assert servo[key] in links, key
