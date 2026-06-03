"""ROS bridge tests: /rosout level maps to the right palette severity."""

from rcl_interfaces.msg import Log

from fm_tui.ros import severity_for


def test_known_levels_map_to_severities():
    assert severity_for(Log.DEBUG) == "debug"
    assert severity_for(Log.INFO) == "info"
    assert severity_for(Log.WARN) == "warn"
    assert severity_for(Log.ERROR) == "error"


def test_fatal_folds_into_error():
    assert severity_for(Log.FATAL) == "error"


def test_unknown_level_falls_back_to_info():
    assert severity_for(0) == "info"
