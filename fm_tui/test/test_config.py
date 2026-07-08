"""Config tests: path resolution, round-trip, and viewer validation.

No ROS, no Textual — the config module is plain JSON over a file path, so these
run anywhere. Each test points ``FM_TUI_CONFIG`` at a tmp file so the working
tree's real ``.fm_tui.json`` is never read or written.
"""

from pathlib import Path

from fm_tui import config


def test_path_prefers_env_over_cwd(monkeypatch, tmp_path):
    target = tmp_path / "somewhere" / "cfg.json"
    monkeypatch.setenv("FM_TUI_CONFIG", str(target))
    assert config.config_path() == target


def test_path_falls_back_to_cwd(monkeypatch, tmp_path):
    monkeypatch.delenv("FM_TUI_CONFIG", raising=False)
    monkeypatch.chdir(tmp_path)
    assert config.config_path() == tmp_path / ".fm_tui.json"


def test_missing_file_yields_default(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "absent.json"))
    assert config.get_viewer() == "foxglove"
    assert config.load() == {"viewer": "foxglove"}


def test_save_then_load_round_trips(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    config.set_viewer("rviz")
    assert config.get_viewer() == "rviz"
    assert config.load()["viewer"] == "rviz"


def test_set_viewer_preserves_other_keys(monkeypatch, tmp_path):
    cfg = tmp_path / "cfg.json"
    monkeypatch.setenv("FM_TUI_CONFIG", str(cfg))
    config.save({"viewer": "foxglove", "robot": "g1_d"})
    config.set_viewer("rviz")
    assert config.load() == {"viewer": "rviz", "robot": "g1_d"}


def test_unknown_viewer_in_file_falls_back(monkeypatch, tmp_path):
    cfg = tmp_path / "cfg.json"
    monkeypatch.setenv("FM_TUI_CONFIG", str(cfg))
    config.save({"viewer": "weird"})
    assert config.get_viewer() == "foxglove"


def test_corrupt_file_falls_back(monkeypatch, tmp_path):
    cfg = tmp_path / "cfg.json"
    cfg.write_text("{not valid json")
    monkeypatch.setenv("FM_TUI_CONFIG", str(cfg))
    assert config.get_viewer() == "foxglove"


def test_set_viewer_rejects_unknown(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    try:
        config.set_viewer("bogus")
    except ValueError:
        return
    raise AssertionError("expected ValueError for unknown viewer")


def test_camera_defaults_when_absent(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "absent.json"))
    assert config.get_camera() == {"camera": "phone", "phone_ip": ""}


def test_set_camera_round_trips_phone_with_ip(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    config.set_camera("phone", "192.168.1.207:8081")
    assert config.get_camera() == {"camera": "phone", "phone_ip": "192.168.1.207:8081"}


def test_set_camera_mac_clears_nothing_else(monkeypatch, tmp_path):
    cfg = tmp_path / "cfg.json"
    monkeypatch.setenv("FM_TUI_CONFIG", str(cfg))
    config.save({"viewer": "rviz"})
    config.set_camera("mac")
    data = config.load()
    assert data["camera"] == "mac"
    assert data["phone_ip"] == ""
    assert data["viewer"] == "rviz"  # camera write preserves the viewer key


def test_get_camera_falls_back_on_unknown_value(monkeypatch, tmp_path):
    cfg = tmp_path / "cfg.json"
    monkeypatch.setenv("FM_TUI_CONFIG", str(cfg))
    config.save({"camera": "webcam", "phone_ip": "10.0.0.5"})
    # Unknown camera value falls back to the default, but the stored IP is kept.
    assert config.get_camera() == {"camera": "phone", "phone_ip": "10.0.0.5"}


def test_set_camera_rejects_unknown(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    try:
        config.set_camera("hologram")
    except ValueError:
        return
    raise AssertionError("expected ValueError for unknown camera")
