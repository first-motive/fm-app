"""Guards on the viewer page's one security-relevant invariant.

The page renders strings that arrive over the websocket — session names, error text,
servo and clamp keys, sensor device names — into ``innerHTML``. The author knew this
and wrote ``esc()`` for it. The gap was that ``metric()`` interpolated its arguments
raw, and one call site (``metric("Rows", s.rows || 0)``) passed a field straight off
``/capture/*``, so a string there reached the DOM unescaped.

The fix moved escaping into ``metric()`` itself, which means the invariant is now
"the helper escapes, callers pass plain strings". These tests hold both halves of
that, because the failure is invisible in review: a new ``metric()`` call reads
exactly like a safe one.

Static checks on purpose. The ros:humble container that runs colcon test has no
JavaScript engine, and a test that needs node would either not run here or run
somewhere it cannot fail the build.
"""

import re
from pathlib import Path

import pytest

INDEX = Path(__file__).resolve().parents[1] / "webgui" / "index.html"
SOURCE = INDEX.read_text()

METRIC_DEFINITION = re.search(
    r"const metric = \(label, val, small\) =>\n(?P<body>.*?);\n", SOURCE, re.DOTALL
)


def test_metric_definition_is_where_the_tests_think_it_is():
    """A rename should fail loudly here, not silently skip the guard below."""
    assert METRIC_DEFINITION, "metric() definition not found — update this test with it"


@pytest.mark.parametrize("argument", ["label", "val"])
def test_metric_escapes_both_arguments(argument):
    body = METRIC_DEFINITION.group("body")
    assert f"esc({argument})" in body, (
        f"metric() interpolates {argument} without esc(). Wire fields reach this "
        f"cell, so an unescaped argument is a script injection from any peer that "
        f"can publish /capture/*."
    )


def test_no_metric_call_site_pre_escapes_its_argument():
    """Double escaping is not a hole, it is a visible bug: `&amp;` shown to the operator."""
    offenders = [
        stripped
        for stripped in (line.strip() for line in SOURCE.splitlines())
        # Call sites only: skip the definition, and skip comments, which discuss
        # esc() precisely because this rule exists.
        if stripped.startswith("metric(") and "esc(" in stripped
    ]
    assert not offenders, (
        "metric() escapes its own arguments; these call sites escape again: " f"{offenders}"
    )


def test_esc_covers_the_dangerous_characters():
    """An esc() that misses a quote is worse than none — it reads as protection."""
    definition = re.search(r"const esc = .*?\}\[c\]\)\);", SOURCE, re.DOTALL)
    assert definition, "esc() definition not found — update this test with it"
    for char in ("&", "<", ">", '"', "'"):
        assert f'"{char}"' in definition.group() or f"'{char}'" in definition.group(), (
            f"esc() does not map {char!r}"
        )
