# fm_app

Application layer for the fm_ros2 workspace. Groups the launch orchestration and
the operator TUI — the user-facing entry points that start and drive the stack.
Split-ready: this whole group extracts cleanly into its own repo later.

## Sub-Packages

| Package | Build | Role |
|---------|-------|------|
| [`fm_bringup`](fm_bringup/README.md) | ament_python | Top-level launch files and config that compose the full stack (real and sim) |
| [`fm_tui`](fm_tui/README.md) | ament_python | Operator terminal UI: the launcher console_script that drives bringup |

## How the Pieces Connect

`fm_tui` is the entry point the operator runs; it launches `fm_bringup`, which
composes the controllers, sensors, and (in sim) the chosen engine into a running
stack. `fm_bringup` is the orchestration seam — it pulls the robot, sim, and control
layers together through their launch files. See
[docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md) for the full launch graph.

## Build Type

`ament_cmake` metapackage (exec-depends on the two sub-packages). The package
itself builds nothing; it ties the group together for a single install.
