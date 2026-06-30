# fm_tui

Two terminal UIs over the ROS2 stack, Python (Textual):

| Mode | Entry | Does |
|------|-------|------|
| **monitor** | `fm_tui` | watch the live graph — nodes, topics, `/rosout` |
| **launcher** | `fm_tui_launcher` | pick a launch from a menu and dispatch it |

## Monitor

```
ROS2 graph ──► fm_tui (rclpy) ──► terminal UI
  /rosout, node + topic lists       severity-coloured panels
```

```bash
ros2 run fm_tui fm_tui
```

Press `q` to quit. The UI needs a real terminal; it does not render under a
pipe or in CI.

## Launcher Mode

The launcher is the menu behind the repo-root `./run.sh`. It walks a declarative
registry — action → robot → variant — then dispatches the matching launch:

```
registry.py (data) ──► launcher.py (Textual menu) ──► ros2 launch …
  actions, robots, variants    arrow-key walk          wired action only
```

```bash
ros2 run fm_tui fm_tui_launcher
```

Selecting a variant exits the UI and hands the terminal to the launch. Today
Robot Description is wired (to `fm_description view_robot.launch.py`); Teleop and
Autonomous render as disabled stubs until their launch graphs land.

`registry.py` is the single source of truth for the **menu**; the launch file
owns the **dispatch params**. `scripts/view-robot.sh` drives the same launch file
from the host as the direct, scriptable path — the launcher and the script are
two doors onto one launch file.

## Layout

```
◢ FIRST MOTIVE · FM_TUI — ROS2 MONITOR        ROS2 ● LIVE · 12 nodes
┏━ NODES · 12 ━━━━━━━┓ ┏━ TOPICS · 34 ━━━━┓
┃ /controller_node  ┃ ┃ /joint_states    ┃
┗━━━━━━━━━━━━━━━━━━━━┛ ┗━━━━━━━━━━━━━━━━━━┛
┏━ /ROSOUT ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ 12:04:51 ● INFO  bringup ready          ┃
┃ 12:04:52 ▲ WARN  battery low            ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
 [Q] QUIT   [↑↓] MOVE
```

A branded status bar tops the monitor: brand mark left, live ROS link right
(`ROS2 ● LIVE · N nodes`, or `ROS2 ○ OFFLINE` before the graph connects). Panels
badge their live counts (`NODES · 12`); `/rosout` aligns each line as a dim
timestamp, severity glyph, severity, then the message.

Severity glyphs: `info ●` · `warn ▲` · `error ✕` · `debug ·`.

## Theming — nish-tui (optional, recommended)

[nish-tui](https://github.com/ubunish/nish-tui) is a soft dependency. fm_tui
detects it at import time and picks a widget set accordingly:

```
import nish_tui succeeds ──► themed widgets + nish-tui palette
import nish_tui fails    ──► plain fallback twins (stock terminal colours)
```

Either way fm_tui runs and stays readable. Installing nish-tui is recommended
for the cleaner, consistent palette shared with the rest of the stack:

```bash
pip install nish-tui
```

No configuration follows — the swap is automatic. The resolver lives in
`fm_tools.tui.theme`; the fallback twins in `fm_tools.tui.widgets` mirror the
nish-tui widget API so the app code never branches on availability. The fallback
twins carry the First Motive palette (`fm_tools.tui.palette`), the same source the
run.sh step banners paint from, so the bare TUI stays on-brand. These now live in
the shared `fm-tools` wheel, not in `fm_tui`.

## Terminal Font

fm_tui sets its brand colour, glyphs, and layout — but not the typeface. A
terminal app draws with whatever font the terminal emulator is configured to
use; it cannot ship its own. For the on-brand match with the
[d2 diagrams](https://github.com/first-motive/fm-ros2/tree/main/docs/diagrams),
set your terminal profile to **Geist Mono** (the First Motive brand mono). Any
monospace font with box-drawing and symbol coverage renders the heavy borders
and severity glyphs (`◢ ● ▲ ✕ · ┏ ━`) cleanly.

## Build Type

`ament_python`. Depends on `rclpy` and `rcl_interfaces`; `textual` comes from pip.
