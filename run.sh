#!/usr/bin/env bash
# Standalone front door for fm-app. Builds the workspace and opens the fm_tui
# launcher — an arrow-key menu that picks the action, robot, and backend itself,
# then dispatches the launch. The launcher drives every backend, so this repo
# takes no --robot flag: the TUI does the selecting.
#
# The host OS picks the path (override with --native / --container):
#   Linux  -> native:    build + launch directly on the host (ROS2 Humble + the
#                        full sim/teleop deps must be installed)
#   Darwin -> container: build the fm-app image, bring it up via the fm-docker
#                        compose overlays, build + launch inside it (OrbStack)
#
#   ./run.sh                       # auto-detect, open the launcher
#   ./run.sh --native              # force the host path (Linux)
#   ./run.sh --container           # force the container path (macOS / OrbStack)
#   ./run.sh --no-foxglove         # extra args pass through to the launcher
set -euo pipefail

cd "$(dirname "$0")"

# --- Per-repo config (downstream repos retune these two) ----------------------
IMAGE=fm-app:humble                            # local image tag for the container path
LAUNCH=(ros2 run fm_tui fm_tui_launcher)       # the interactive TUI launcher
# -----------------------------------------------------------------------------

MODE=""                  # "" = auto-detect; else native | container
PASSTHROUGH=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --native)    MODE=native; shift ;;
    --container) MODE=container; shift ;;
    *)           PASSTHROUGH+=("$1"); shift ;;
  esac
done

# Auto-detect the path from the host OS when not forced by a flag.
if [[ -z "$MODE" ]]; then
  case "$(uname -s)" in
    Linux)  MODE=native ;;
    Darwin) MODE=container ;;
    *) echo "error: unsupported host '$(uname -s)' — pass --native or --container" >&2; exit 1 ;;
  esac
fi

LAUNCH+=(${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"})

if [[ "$MODE" == native ]]; then
  # Host path: pull siblings + externals once, build in place, open the launcher.
  set +u  # ROS setup scripts reference unbound vars; nounset would abort the source
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
  if [[ ! -d external ]]; then
    vcs import < fm-app.repos
  fi
  rosdep install --from-paths . external --ignore-src -y -r 2>/dev/null || true
  colcon build --symlink-install
  set +u
  source install/setup.bash
  set -u
  echo ">> opening the fm_tui launcher on the host"
  exec "${LAUNCH[@]}"
fi

# Container path: build the local image, bring it up, build + launch inside it.
# The fm-docker compose overlays live in docker/, imported via fm-app.repos —
# pull them on first run so a fresh clone works with no manual setup.
if [[ ! -d docker ]]; then
  vcs import < fm-app.repos
fi
COMPOSE=(docker compose -f docker/compose.yaml -f docker/compose.macos.yaml)
export FM_IMAGE="$IMAGE"
export FM_WS="$PWD"

echo ">> building $IMAGE (FROM the fm-robot layer)"
docker build -t "$IMAGE" .
echo ">> bringing the container up (idempotent)"
"${COMPOSE[@]}" up -d
echo ">> building the workspace inside the container"
"${COMPOSE[@]}" exec fm /ros_entrypoint.sh colcon build --symlink-install
echo ">> opening the fm_tui launcher"
echo ">> tear down with: ${COMPOSE[*]} down"
# The launcher is an interactive TUI — it needs a tty, so route through an
# interactive `exec` (no -T). `exec` skips the image ENTRYPOINT, so go through
# /ros_entrypoint.sh to source ROS + the workspace overlay.
exec "${COMPOSE[@]}" exec fm /ros_entrypoint.sh "${LAUNCH[@]}"
