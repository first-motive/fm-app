# fm_bringup

Launch files and runtime configs for the First Motive graph: the original stub
bringup, plus the OpenArm, SO101, and G1-D simulation and teleop stacks.
`ament_python`.

## Robot Registry

The control launch layer is registry-driven (`fm_bringup/registry.py`). One
`RobotSpec` per robot owns everything the launches vary on — the backend-selectable
xacro, the controller sets, which backends need a standalone controller_manager, the
foxglove_bridge params, and the MoveIt Servo context (SRDF / kinematics / servo.yaml).
`sim.launch.py` / `servo.launch.py` / `teleop.launch.py` dispatch through
`registry.get(robot)` and hold no robot-specific data. Adding a robot is one entry
here, mirroring `fm_description`'s `view_robot.launch.py`.

Registered: `openarm`, `so101`, `g1_d`.

## Launches

```
launch/
  bringup.launch.py             foxglove_bridge + control stub
  sim.launch.py                 unified sim: robot + variant + sim_backend
  servo.launch.py               MoveIt Servo for the robot's arm group
  teleop.launch.py              Servo + selected input (foxglove | joy | spacenav)
  controllers.launch.py         reusable controller spawners (+ optional standalone CM)
```

The per-backend launch hosts (mujoco / gazebo / isaac) live in
[`fm_sim_backends`](../fm_sim/fm_sim_backends); `sim.launch.py` includes one of them
by name based on `sim_backend`.

`sim.launch.py` builds the description from the robot's backend-selectable xacro,
starts `robot_state_publisher` + `foxglove_bridge`, brings up the controller_manager
for the chosen backend, then spawns the controllers. The robot's
`standalone_cm_backends` decides which backends need a standalone `ros2_control_node`
(mock/real for OpenArm + SO101; mock only for the G1-D, whose real arm is a bridge,
not a controller_manager); mujoco/gazebo host the controller_manager inside the sim;
isaac bridges to an externally-running Isaac Sim over ROS topics.

```bash
ros2 launch fm_bringup sim.launch.py robot:=so101 sim_backend:=mujoco
ros2 launch fm_bringup teleop.launch.py robot:=g1_d input:=foxglove sim_backend:=mock
```

Prefer `scripts/sim.sh` and `scripts/teleop.sh` from the host — they pick the compose
overlay from the backend.

## Configs

```
config/<robot>/
  <variant>.controllers.yaml    jsb + arm JTC (+ gripper JTC where the robot has one)
  <robot>.srdf                  Servo's planning group (in-repo, Humble)
  kinematics.yaml               KDL solver for the arm group
  joint_limits.yaml             Servo's velocity/accel caps
  servo.yaml                    MoveIt Servo params (group, frames, command topic)
```

`config/openarm` reuses the vendored OpenArm MoveIt config for kinematics + limits;
`config/so101` and `config/g1_d` carry an in-repo MoveIt config authored for Humble.
Per-robot Servo reach: OpenArm + G1-D are 7-DOF (full 6-DOF Cartesian); SO101 is 5-DOF
(JointJog primary, Cartesian translation-only — orientation drifts on the unreachable
axis). The controller set is identical across sim backends; only the `<ros2_control>`
System plugin in the description swaps. Joint names match each description's geometry.

## Teleop Input

The teleop source nodes live in `fm_teleop_device` (gamepad, SpaceMouse, G1-D hand);
`teleop.launch.py` here orchestrates them — it spawns the selected source alongside its
driver node (`joy_node` / `spacenav_node`) and MoveIt Servo.

The primary input is the browser-side Foxglove panel (`fm_teleop/fm_teleop_panel/`);
the device sources cover physical-HID inputs on Linux hosts. See `fm_teleop/` for the
source layer and the command contract.

## Build Type

`ament_python`.
