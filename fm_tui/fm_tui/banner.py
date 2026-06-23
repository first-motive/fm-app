"""FM-branded terminal banners — the colour-coded step rules run.sh paints with.

run.sh narrates each launch step (detect OS, bring the container up, build the
workspace, open the TUI). Each step renders as a numbered header block — a rule,
the ``N. title``, another rule — in the First Motive palette. run.sh prints the
per-step status as plain ``- `` bullets beneath the block, so a run reads as::

    ────────────────────────────────────────
    1. Detect OS
    ────────────────────────────────────────
    - macOS detected

    ────────────────────────────────────────
    2. macOS Container
    ────────────────────────────────────────
    - OrbStack already installed
    - Container up

The rules are drawn by rich's ``Console.rule`` command, which fits the line to the
terminal width for us. The palette lives here once so run.sh and the TUI share one
source of brand colour (it mirrors ``docs/diagrams/styles.d2``).

rich is a fm_tui dependency (Textual already pulls it in). The first run.sh steps
fire on the host before the container exists, so run.sh runs this through
``uv run --with rich`` to get rich on the host too.

    python3 -m fm_tui.banner 1 "Detect OS"
    python3 -m fm_tui.banner 4 "Launcher" info
"""

from __future__ import annotations

import sys

from rich.console import Console
from rich.style import Style
from rich.text import Text

# First Motive palette — mirrors docs/diagrams/styles.d2.
PLUM = "#3B3443"
LILAC = "#B6A5C6"
SAND = "#E7DDC8"
CREAM = "#ECE2CF"

# Role -> colour. ``step`` is an active launch step; ``info`` a secondary note
# (endpoints, teardown hint); ``done`` a completed milestone.
ROLES = {
    "step": LILAC,
    "info": SAND,
    "done": CREAM,
}


def emit(number, title: str, role: str = "step", *, console: Console | None = None) -> None:
    """Draw a numbered step header block (rule / ``N. title`` / rule) to ``console``.

    Defaults to a stdout console, which auto-detects width, TTY, and ``NO_COLOR``.
    """
    console = console or Console()
    colour = ROLES.get(role, LILAC)
    console.rule(style=colour)
    console.print(Text(f"{number}. {title}", style=Style(color=colour, bold=True)))
    console.rule(style=colour)


def main(argv: list[str] | None = None) -> int:
    """CLI: ``banner.py <number> <title> [role]`` — print one step header block."""
    args = sys.argv[1:] if argv is None else argv
    if len(args) < 2:
        print("usage: banner.py <number> <title> [step|info|done]", file=sys.stderr)
        return 2
    emit(args[0], args[1], args[2] if len(args) > 2 else "step")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
