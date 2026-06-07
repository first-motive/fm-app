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
