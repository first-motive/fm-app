# fm_tui

A single-screen ROS2 terminal monitor. Python (Textual). Shows live nodes,
topics, and colour-coded `/rosout` logs.

## Role

```
ROS2 graph ──► fm_tui (rclpy) ──► terminal UI
  /rosout, node + topic lists       severity-coloured panels
```

## Run

```bash
ros2 run fm_tui fm_tui
```

Press `q` to quit. The UI needs a real terminal; it does not render under a
pipe or in CI.

## Layout

```
Header
┌ nodes ──────┐ ┌ topics ─────┐
└─────────────┘ └─────────────┘
┌ /rosout ────────────────────┐   debug·grey  info·green  warn·amber  error·red
└──────────────────────────────┘
Footer
```

## Theming — nish-tui (optional, recommended)

[nish-tui](https://github.com/nishalan/nish-tui) is a soft dependency. fm_tui
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
`fm_tui/theme.py`; the fallback twins in `fm_tui/widgets.py` mirror the nish-tui
widget API so the app code never branches on availability.

## Build type

`ament_python`. Depends on `rclpy` and `rcl_interfaces`; `textual` comes from pip.
