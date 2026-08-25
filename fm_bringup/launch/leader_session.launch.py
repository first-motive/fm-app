"""One-command leader-follower session: sim (or real target) plus the leader source.

    sim.launch.py                 robot description, controllers, foxglove_bridge
    teleop.launch.py input:=leader  leader_source -> the arm controller (no Servo)

The leader path bypasses MoveIt Servo, so this session brings up no servo_node — the
source streams a trajectory straight to the follower's arm controller and owns the motion
bounds itself (deadman, per-period ramp, stale-stream stop; see fm_teleop_leader).

Teleop is held back by ``teleop_delay`` so the controllers and ``/joint_states`` are up
first: the source seeds its ramp from the follower's current joint state, and engaging
before that state arrives just holds.

In sim any publisher on ``leader_topic`` stands in for the arm, which is how the mode is
proven on the mujoco backend before a leader is plugged in. On hardware, run
``fm_teleop_leader/leader_driver`` beside this session.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from fm_bringup.sessions import include as _include


def _launch_setup(context, *args, **kwargs):
    common = {
        name: LaunchConfiguration(name).perform(context)
        for name in ("robot", "variant", "sim_backend")
    }
    teleop_delay = float(LaunchConfiguration("teleop_delay").perform(context))

    sim = _include(
        "sim.launch.py",
        {**common, "use_foxglove": LaunchConfiguration("use_foxglove").perform(context)},
    )
    teleop = TimerAction(
        period=teleop_delay,
        actions=[
            _include(
                "teleop.launch.py",
                {
                    **common,
                    "input": "leader",
                    "leader_topic": LaunchConfiguration("leader_topic").perform(context),
                    "leader_arm": LaunchConfiguration("leader_arm").perform(context),
                },
            )
        ],
    )
    return [sim, teleop]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="so101"),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description="Empty uses the registry default for the robot.",
            ),
            DeclareLaunchArgument("sim_backend", default_value="mujoco"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument(
                "leader_topic",
                default_value="/leader/joint_states",
                description="The leader arm's JointState topic.",
            ),
            DeclareLaunchArgument(
                "leader_arm",
                default_value="",
                description="Which arm controller the leader drives on a multi-arm "
                "variant. Empty takes the first.",
            ),
            DeclareLaunchArgument(
                "teleop_delay",
                default_value="8.0",
                description="Seconds to wait for the sim's controllers before starting "
                "the source.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
