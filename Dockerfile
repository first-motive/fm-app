# fm-app image — the full-stack launcher image, FROM the robot layer.
#
# fm-app is the integration repo: its TUI launcher drives every backend, so this
# image reconverges the union of the sim and teleop apt deps on top of the robot
# layer rather than expecting a separate image per backend. Downstream, the
# orchestrator (fm-ros2) consumes this image. The entrypoint, WORKDIR, and the
# robot/viz tooling are inherited from fm-robot — this layer only adds the sim +
# teleop packages plus the TUI's Python deps.
FROM ghcr.io/first-motive/fm-robot:humble

ARG DEBIAN_FRONTEND=noninteractive

# packages.ros.org currently ships libignition-gazebo6 6.18.0 (which needs
# libignition-sensors6 >= 6.8.1) but only libignition-sensors6 6.8.0, so the gz
# stack will not resolve from ros.org alone. Add the OSRF Gazebo repo, which
# carries the matching 6.8.1 sensors libs. Drop this once ros.org is consistent.
RUN apt-get update && apt-get install -y --no-install-recommends \
      wget gnupg lsb-release \
    && wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
         https://packages.osrfoundation.org/gazebo.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# Sim + teleop union: the MuJoCo and Gazebo ros2_control plugins, the ros_gz
# bridge/sim pair, MoveIt + servo for teleop, and the headless GL stack (xvfb +
# mesa) the simulators render against. All on the Humble apt mirror (gz libs via
# the OSRF repo above) for both arm64 and amd64, so no source builds.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-mujoco-ros2-control \
      ros-humble-gz-ros2-control \
      ros-humble-ros-gz-sim \
      ros-humble-ros-gz-bridge \
      ros-humble-moveit \
      ros-humble-moveit-servo \
      xvfb \
      libgl1-mesa-dri \
      libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Python deps colcon does not resolve: the MuJoCo physics engine the sim core
# drives, textual — fm_tui's TUI framework — and fm-tools, the shared wheel that
# carries fm_tui's brand, widgets, and pick menu (SHA-pinned git install == tag
# v0.1.0). The textual pin tracks fm_tui/setup.py (textual==0.74.0); keep the two
# in lockstep, and the fm-tools SHA in lockstep with fm_tui/setup.py.
RUN pip install --no-cache-dir mujoco textual==0.74.0 \
      "fm-tools @ git+https://github.com/first-motive/fm-tools@5d9ef62f9449321730b8ebcacef7be3bc13448f5"

# fm_teleop_vision camera hand-tracking (input:=vision and input:=mirror). Large wheel,
# pulls opencv (cv2) + protobuf/numpy, so it caches as its own layer. Pin needs an arm64
# cp310 wheel (Ubuntu 22.04 / Python 3.10 on the linux/arm64 Mac target).
RUN pip install --no-cache-dir mediapipe==0.10.14
