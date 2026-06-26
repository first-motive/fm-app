#!/usr/bin/env bash
# Standalone front door for fm-app. Builds the workspace and opens the fm_tui
# launcher — an arrow-key menu that picks the action, robot, and backend itself,
# then dispatches the launch. The launcher drives every backend, so this repo
# takes no --robot/--backend flag: the TUI does the selecting.
#
# Curl-able (no clone needed) on macOS:
#   curl -fsSL https://raw.githubusercontent.com/first-motive/fm-app/main/run.sh | bash
#
# From a clone:
#   ./run.sh [--native|--container] [-h|--help] [launcher args…]
#
# The host OS picks the path (override with --native / --container):
#   linux  -> native:    build + launch directly on the host (ROS2 Humble + the
#                        full sim/teleop deps must be installed)
#   macos  -> container: run the fm-app image via the fm-docker compose overlays,
#                        build + launch inside it (OrbStack)
#
# Piped via curl, the shared host checks (fm-tools lib.sh) and the compose
# overlays are fetched from their pinned tags and cached under ~/.cache/fm-app,
# so later runs work offline.
#
#   ./run.sh                       # auto-detect path, open the launcher
#   ./run.sh --native              # force the host path (Linux)
#   ./run.sh --container           # force the container path (macOS / OrbStack)
#   ./run.sh --no-foxglove         # extra args pass through to the launcher
#
# The body is wrapped in main() and called on the last line, so a truncated
# curl|bash never half-runs.
set -euo pipefail

# --- Per-repo config (downstream repos retune these) --------------------------
LOCAL_IMAGE=fm-app:humble                          # locally-built tag for the clone dev loop
BAKED_IMAGE=ghcr.io/first-motive/fm-app:humble     # published image for the no-clone baked path
LAUNCH=(ros2 run fm_tui fm_tui_launcher)           # the interactive TUI launcher
FM_APP_RAW="https://raw.githubusercontent.com/first-motive/fm-app/main"
# lib.sh is owned by fm-tools, fetched from a pinned release tag (the single
# reuse home). The container runtime is delegated to fm-docker via install.sh.
FM_TOOLS_RAW="https://raw.githubusercontent.com/first-motive/fm-tools/v0.2.0"
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/fm-app"
# -----------------------------------------------------------------------------

# Keep the caller's directory: it is the workspace for the native build and the
# mount (FM_WS) for the container.
INVOKE_DIR="$PWD"

# Resolve the script's own dir; empty when piped via curl|bash. A clone has the
# repo files next to the script (REPO_DIR set); a piped run does not (REPO_DIR
# empty), so deps are fetched from the raw URLs instead.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-}")" 2>/dev/null && pwd)" || SCRIPT_DIR=""
if [ -n "$SCRIPT_DIR" ] && [ -f "$SCRIPT_DIR/fm-app.repos" ]; then
  REPO_DIR="$SCRIPT_DIR"
else
  REPO_DIR=""
fi

# Load the shared bootstrap library (fm-tools lib.sh) for fm_detect_os /
# fm_has_docker / fm_reattach_tty. Reuse a cached fetch, else fetch from the
# pinned fm-tools tag and cache it. run.sh is itself curl|bash-able, so the
# library may not be on disk. The checks must run in this shell, so source
# rather than execute.
load_lib() {
  local cached="$CACHE_DIR/lib.sh"
  if [ ! -f "$cached" ]; then
    mkdir -p "$CACHE_DIR"
    chmod 700 "$CACHE_DIR"  # lib.sh is sourced from here; keep the cache user-only
    # Fetch to a temp file and rename only on success: an interrupted download
    # must never leave a partial file later runs treat as cached.
    local tmp="$cached.tmp.$$"
    curl -fsSL --proto '=https' --proto-redir '=https' "$FM_TOOLS_RAW/lib.sh" -o "$tmp" \
      || { rm -f "$tmp"; echo "error: failed to fetch lib.sh from fm-tools" >&2; exit 1; }
    [ -s "$tmp" ] || { rm -f "$tmp"; echo "error: empty lib.sh download" >&2; exit 1; }
    mv "$tmp" "$cached"
  fi
  # shellcheck source=/dev/null
  source "$cached"
}

usage() {
  cat <<'EOF'
run.sh — build the workspace and open the fm_tui launcher

Usage: ./run.sh [--native|--container] [-h|--help] [launcher args…]

  --native         force the host path (Linux)
  --container      force the container path (macOS / OrbStack)
  -h, --help       show this help

The launcher is an arrow-key TUI that selects the action, robot, and backend.
Extra args (e.g. --no-foxglove) pass through to it.
Env: FM_SELFTEST=1  load deps + resolve OS/mode, then stop before any work.
EOF
}

main() {
  local mode=""
  local -a passthrough=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)   usage; return 0 ;;
      --native)    mode=native; shift ;;
      --container) mode=container; shift ;;
      *)           passthrough+=("$1"); shift ;;
    esac
  done

  load_lib

  # Auto-detect the path from the host OS when not forced by a flag. fm_detect_os
  # (from lib.sh) echoes macos|linux.
  if [[ -z "$mode" ]]; then
    case "$(fm_detect_os)" in
      linux)  mode=native ;;
      macos)  mode=container ;;
      *) echo "error: could not resolve host path — pass --native or --container" >&2; return 1 ;;
    esac
  fi

  # CI self-test hook: deps loaded, OS + mode resolved — stop before any runtime
  # work. Lets the curl-path test exercise the piped fetch without OrbStack.
  if [ -n "${FM_SELFTEST:-}" ]; then
    echo "selftest ok: lib loaded, mode=$mode"
    return 0
  fi

  # Passthrough args (e.g. --no-foxglove) reach the launcher.
  LAUNCH+=(${passthrough[@]+"${passthrough[@]}"})

  if [[ "$mode" == native ]]; then
    run_native
  else
    run_container
  fi
}

# Host path: pull siblings + externals once, build in place, open the launcher.
run_native() {
  set +u  # ROS setup scripts reference unbound vars; nounset would abort the source
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
  cd "$INVOKE_DIR"
  if [[ ! -d external ]]; then
    vcs import < fm-app.repos
  fi
  rosdep install --from-paths . external --ignore-src -y -r 2>/dev/null || true
  colcon build --symlink-install
  set +u
  # shellcheck source=/dev/null
  source install/setup.bash
  set -u
  echo ">> opening the fm_tui launcher on the host"
  exec "${LAUNCH[@]}"
}

# Container path (macOS / OrbStack). Bring up a runtime if none is present, then
# dispatch on clone vs pipe:
#   pipe (no source on disk) -> pull the baked image and run it with no mount, so
#                               the entrypoint sources the workspace baked in.
#   clone (source on disk)   -> mount source at /ws, rebuild inside, launch, so
#                               edits override the baked build (the dev loop).
run_container() {
  cd "$INVOKE_DIR"

  # Bring up a container runtime if missing — install + start OrbStack via install.sh.
  if ! fm_has_docker; then
    echo ">> no container runtime — setting up OrbStack via install.sh"
    if [ -n "$REPO_DIR" ]; then
      bash "$REPO_DIR/install.sh" --no-pull
    else
      curl -fsSL --proto '=https' --proto-redir '=https' "$FM_APP_RAW/install.sh" | bash -s -- --no-pull
    fi
    fm_has_docker || { echo "error: container runtime still unavailable after setup." >&2; return 1; }
  fi

  if [ -z "$REPO_DIR" ]; then
    # Baked path: curl-to-launch, no clone, no mount. The image carries a built
    # /ws/install overlay (see Dockerfile), so route through the entrypoint to
    # source ROS + that overlay, then launch. --pull missing fetches on first run;
    # arm64 matches the macOS overlay's platform pin.
    #
    # The launcher is an interactive TUI — it needs a terminal. Piped as
    # `curl … | bash`, this script's stdin is the pipe, not the keyboard, so
    # reattach the controlling tty (fm_reattach_tty, from lib.sh) and run with
    # `-i` reading from it, so the menu is actually usable.
    echo ">> running the baked image $BAKED_IMAGE (no clone, no mount)"
    echo ">> opening the fm_tui launcher"
    fm_reattach_tty
    exec docker run --rm -i --pull missing --platform linux/arm64 \
      "$BAKED_IMAGE" /ros_entrypoint.sh "${LAUNCH[@]}"
  fi

  # Mounted path: build the local image, bring it up, build + launch inside it.
  # The fm-docker compose overlays live in docker/, imported via fm-app.repos —
  # pull them on first run so a fresh clone works with no manual setup.
  if [[ ! -d docker ]]; then
    vcs import < fm-app.repos
  fi
  local -a compose=(docker compose -f docker/compose.yaml -f docker/compose.macos.yaml)
  export FM_IMAGE="$LOCAL_IMAGE"
  export FM_WS="$INVOKE_DIR"

  echo ">> building $LOCAL_IMAGE (FROM the fm-robot layer)"
  docker build -t "$LOCAL_IMAGE" .
  echo ">> bringing the container up (idempotent)"
  "${compose[@]}" up -d
  echo ">> building the workspace inside the container"
  "${compose[@]}" exec fm /ros_entrypoint.sh colcon build --symlink-install
  echo ">> opening the fm_tui launcher"
  echo ">> tear down with: ${compose[*]} down"
  # The launcher is an interactive TUI — it needs a tty, so route through an
  # interactive `exec` (no -T). `exec` skips the image ENTRYPOINT, so go through
  # /ros_entrypoint.sh to source ROS + the workspace overlay.
  exec "${compose[@]}" exec fm /ros_entrypoint.sh "${LAUNCH[@]}"
}

main "$@"
