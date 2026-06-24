from visualization_msgs.msg import Marker

from fm_bringup.task_env_markers import MARKER_TOPIC, markers_for_task_env


def test_pick_place_markers_include_table_cube_and_goal():
    array = markers_for_task_env("pick_place")
    assert [marker.type for marker in array.markers] == [
        Marker.CUBE,
        Marker.CUBE,
        Marker.CYLINDER,
    ]
    assert [marker.header.frame_id for marker in array.markers] == ["base_link", "base_link", "base_link"]
    assert array.markers[1].pose.position.y == -0.07
    assert array.markers[2].scale.z == 0.006


def test_bin_sort_markers_cover_bins_and_both_cubes():
    array = markers_for_task_env("bin_sort")
    assert len(array.markers) == 5
    names = [(marker.color.r, marker.color.g, marker.color.b) for marker in array.markers]
    assert (0.95, 0.82, 0.12) in names
    assert (0.12, 0.78, 0.78) in names


def test_unknown_env_yields_empty_marker_array():
    array = markers_for_task_env("default")
    assert array.markers == []
