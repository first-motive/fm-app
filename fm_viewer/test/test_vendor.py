"""Integrity tests for the vendored JavaScript.

These are the files no one reviews: five third-party libraries, 940K, checked in and
served straight to an operator's browser. Nothing in the repo recorded what they
were, so "which three.js is this, and does that CVE apply?" had no answer short of
reverse-engineering the minified bundle.

``webgui/vendor/vendor.json`` records the answer, and these tests keep it true. They
are offline by design — they compare the manifest to the bytes on disk, so they run
in the same colcon test pass as everything else and cannot go red because unpkg is
down.
"""

import hashlib
import json
from pathlib import Path

import pytest

VENDOR_DIR = Path(__file__).resolve().parents[1] / "webgui" / "vendor"
MANIFEST = VENDOR_DIR / "vendor.json"


def _libraries():
    return json.loads(MANIFEST.read_text())["libraries"]


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize("lib", _libraries(), ids=lambda lib: lib["file"])
def test_vendored_file_matches_its_recorded_hash(lib):
    """A hand-edited or silently replaced library is a red build.

    A patched vendored file is the failure mode with no other detector: it looks
    identical in a diff review that skips minified JavaScript, and it ships to every
    operator.
    """
    path = VENDOR_DIR / lib["file"]
    assert path.is_file(), f"{lib['file']} is in the manifest but not on disk"
    assert _sha256(path) == lib["sha256"], (
        f"{lib['file']} does not match the hash in vendor.json. If the change is "
        f"intended, update its version, url, and sha256 together."
    )


def test_every_vendored_file_is_accounted_for():
    """A library added without a manifest entry is the gap this whole file closes."""
    on_disk = {p.name for p in VENDOR_DIR.glob("*.js")}
    recorded = {lib["file"] for lib in _libraries()}
    assert on_disk == recorded, (
        f"unrecorded: {sorted(on_disk - recorded)}; missing: {sorted(recorded - on_disk)}"
    )


@pytest.mark.parametrize("lib", _libraries(), ids=lambda lib: lib["file"])
def test_provenance_is_complete(lib):
    """Version and url are what make a CVE question answerable in a minute."""
    for field in ("package", "version", "url", "sha256", "license"):
        assert lib.get(field), f"{lib['file']} is missing {field}"
    assert lib["url"].startswith("https://"), lib["url"]
    assert lib["version"] in lib["url"], (
        f"{lib['file']}: url does not name version {lib['version']}"
    )
