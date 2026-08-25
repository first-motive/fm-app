"""One-command VR session: sim (or real target), Servo, and the VR source.

    sim.launch.py             robot description, controllers, foxglove_bridge
    teleop.launch.py input:=vr  servo.launch.py + vr_source

The VR source jogs through Servo like the gamepad and vision sources, so this session is
the standard teleop composition — the sim, Servo, and one input node. Teleop is held back
by ``teleop_delay`` so Servo finds ``/joint_states`` and the arm's controller on start.

``vr_source`` consumes a VR *bridge* publishing ``PoseStamped`` + ``Joy``; no headset
runtime is launched here. Publishing those two topics by hand is how the mode is proven
on the mujoco backend before a headset is involved.
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
        actions=[_include("teleop.launch.py", {**common, "input": "vr"})],
    )
    return [sim, teleop]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="openarm"),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description="Empty uses the registry default for the robot.",
            ),
            DeclareLaunchArgument("sim_backend", default_value="mujoco"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument(
                "teleop_delay",
                default_value="8.0",
                description="Seconds to wait for the sim's controllers before starting "
                "Servo and the source.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
