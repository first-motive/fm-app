# fm-app

[![License: Proprietary](https://img.shields.io/badge/License-Proprietary-blue.svg)](LICENSE)

Application layer for First Motive's ROS2 stack. Groups the bringup launch
orchestration and the operator TUI — the user-facing entry points that start and
drive the whole stack.

This is the integration repo: `fm_bringup` composes the robot, sim, and teleop
layers, so its `.repos` pulls the
[`fm-robot`](https://github.com/first-motive/fm-robot),
[`fm-sim`](https://github.com/first-motive/fm-sim), and
[`fm-teleop`](https://github.com/first-motive/fm-teleop) sibling repos.

Part of First Motive's ROS2 (Humble) stack. Assembled with all seven package
repos by [`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Role |
|---------|-------|------|
| `fm_bringup` | ament_python | Top-level launch files and config composing the full stack (real and sim) |
| `fm_tui` | ament_python | Operator terminal UI: the launcher that drives bringup |
| `fm_app` | ament_cmake | Metapackage tying the two together for a single install |

## Standalone Build

Clone into a colcon workspace's `src/`, pull the siblings + externals, then build:

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/first-motive/fm-app.git
vcs import < fm-app/fm-app.repos     # siblings (robot, sim, teleop) + externals
cd .. && colcon build --symlink-install
colcon test && colcon test-result --verbose
```

Sibling repos track `main` — fast inner loop while every repo churns together
early; revisit exact-commit pinning at the first release.

## Governance

Owner-free-on-main — see [CONTRIBUTING.md](CONTRIBUTING.md) and
[`.github/CODEOWNERS`](.github/CODEOWNERS).
