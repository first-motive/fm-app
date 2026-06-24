"""First Motive terminal palette — one source of brand colour for run.sh and the TUI.

The core hexes mirror ``docs/diagrams/styles.d2`` so the run.sh step banners
(:mod:`fm_tui.banner`), the launcher, and the monitor all paint from the same
set. The fallback widgets in :mod:`fm_tui.widgets` brand the bare TUI from these
constants; nish-tui carries its own palette when installed.

``SEVERITY`` and the ``AMBER``/``BRICK`` accents are invented against the brand
this pass for the ``/rosout`` log — the core set has no warm or red tone — and
are expected to be reviewed after first render.
"""

from __future__ import annotations

# Core palette — mirrors docs/diagrams/styles.d2.
PLUM = "#3B3443"
LILAC = "#B6A5C6"
SAND = "#E7DDC8"
CREAM = "#ECE2CF"

# Severity accents — warm tones the core brand set lacks. Invented this pass.
AMBER = "#D9B96A"
BRICK = "#C26B6B"

# Severity -> (glyph, colour) for the /rosout log. Glyphs read at a glance;
# colours sit against the plum/lilac brand. debug renders dim (see widgets.py).
SEVERITY = {
    "debug": ("·", SAND),
    "info": ("●", LILAC),
    "warn": ("▲", AMBER),
    "error": ("✕", BRICK),
}

# Banner role -> colour. ``step`` is an active launch step; ``info`` a secondary
# note (endpoints, teardown hint); ``done`` a completed milestone.
ROLES = {
    "step": LILAC,
    "info": SAND,
    "done": CREAM,
}
