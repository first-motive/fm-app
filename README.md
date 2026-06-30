# fm-app

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Application layer for First Motive's ROS2 stack. Groups the bringup launch
orchestration and the operator TUI — the user-facing entry points that start and
drive the whole stack.

This is the integration repo: `fm_bringup` composes the robot, sim, and teleop
layers, so its `.repos` pulls the
[`fm-robot`](https://github.com/first-motive/fm-robot),
[`fm-sim`](https://github.com/first-motive/fm-sim), and
[`fm-teleop`](https://github.com/first-motive/fm-teleop) sibling repos.

Part of First Motive's ROS2 (Humble) stack. Assembled with the four public package
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

## Run

`run.sh` is the standalone front door: it builds the workspace and opens the
`fm_tui` launcher, an arrow-key menu that picks the action, robot, and backend
itself and dispatches the launch. Because the TUI does the selecting, `run.sh`
takes no `--robot` flag. The host OS picks the path, overridable with `--native`
/ `--container`:

```text
Linux  -> native     build + launch on the host (needs ROS2 Humble installed)
Darwin -> container  build the fm-app image, run it via the fm-docker overlays
```

```bash
./run.sh                     # auto-detect, open the launcher
./run.sh --native            # force the host path (Linux)
./run.sh --container         # force the container path (macOS / OrbStack)
```

The container path imports the shared compose overlays from
[`fm-docker`](https://github.com/first-motive/fm-docker) into `docker/` (via
`fm-app.repos`) and builds this repo's `Dockerfile` — the full-stack launcher
image, `FROM` the `fm-robot` layer, reconverging the sim + teleop deps because
the launcher drives every backend. The launcher is an interactive TUI, so the
container path runs it through an interactive `docker compose exec` (a tty). Tear
down the container with
`docker compose -f docker/compose.yaml -f docker/compose.macos.yaml down`.

## Architecture

`fm_tui` is the launcher an operator drives; `fm_bringup` is the composition root
that resolves everything robot-specific and includes the lower layers (robot, sim,
teleop).

![bringup](docs/diagrams/bringup.svg)

Full launch graph, runtime data flow, and visualization:
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Governance

Owner-free-on-main — see [CONTRIBUTING.md](CONTRIBUTING.md) and
[`.github/CODEOWNERS`](.github/CODEOWNERS).
