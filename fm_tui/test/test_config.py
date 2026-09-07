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


# --- transport ---------------------------------------------------------------
#
# The transport is not a preference like the viewer: it is a fact about the
# machine that the launcher can only *request*. These cover the split — what is
# running now comes from the environment run.sh exported, what was asked for
# lives in the config file, and asking for what is already running records
# nothing.


def test_active_transport_reads_the_environment(monkeypatch):
    monkeypatch.setenv("FM_COMMS_PROFILE", "dds-lan")
    assert config.active_transport() == "dds-lan"


def test_active_transport_defaults_when_unset(monkeypatch):
    monkeypatch.delenv("FM_COMMS_PROFILE", raising=False)
    assert config.active_transport() == "zenoh"


def test_active_transport_ignores_an_unknown_value(monkeypatch):
    monkeypatch.setenv("FM_COMMS_PROFILE", "carrier-pigeon")
    assert config.active_transport() == "zenoh"


def test_no_pending_transport_by_default(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    assert config.get_pending_transport() is None


def test_pending_transport_round_trips(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    monkeypatch.setenv("FM_COMMS_PROFILE", "zenoh")
    config.set_pending_transport("dds-lan")
    assert config.get_pending_transport() == "dds-lan"


def test_requesting_the_running_transport_clears_the_request(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    monkeypatch.setenv("FM_COMMS_PROFILE", "zenoh")
    config.set_pending_transport("dds-lan")
    config.set_pending_transport("zenoh")
    assert config.get_pending_transport() is None


def test_pending_transport_preserves_the_viewer(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    monkeypatch.setenv("FM_COMMS_PROFILE", "zenoh")
    config.set_viewer("rviz")
    config.set_pending_transport("dds-lan")
    assert config.get_viewer() == "rviz"


def test_set_pending_transport_rejects_unknown(monkeypatch, tmp_path):
    monkeypatch.setenv("FM_TUI_CONFIG", str(tmp_path / "cfg.json"))
    try:
        config.set_pending_transport("bogus")
    except ValueError:
        return
    raise AssertionError("expected ValueError for unknown transport")
