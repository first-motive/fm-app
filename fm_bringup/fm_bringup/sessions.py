"""Session-launch helper — include one of this package's launch files by name.

Every session launch (vision, leader, VR, and the policy layer's autonomous one) is the
same shape: compose two or three of `fm_bringup`'s own launch files with a dict of
arguments. Resolving the share directory and wrapping the source is the only mechanical
part of that, and it lives here rather than being restated in each session file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(launch_file, arguments):
    """Include ``launch_file`` from ``fm_bringup/launch`` with ``arguments``."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fm_bringup"), "launch", launch_file
            )
        ),
        launch_arguments=arguments.items(),
    )
