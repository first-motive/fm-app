"""Publish simple Foxglove-visible markers for SO101 task environments.

MuJoCo task-env objects live only inside the MJCF today; they are not part of the
robot_description and mujoco_ros2_control does not publish their body poses into ROS.
This node publishes a lightweight visualization_msgs/MarkerArray so Foxglove can show
the table and training props for manual teleop.

The free cube marker reflects the configured START pose only. It does not track MuJoCo
contact dynamics yet because the backend does not expose non-robot body state on ROS.
"""

from dataclasses import dataclass
from typing import Iterable

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


MARKER_TOPIC = "/task_env_markers"


@dataclass(frozen=True)
class Color:
    r: float
    g: float
    b: float
    a: float = 1.0


@dataclass(frozen=True)
class MarkerSpec:
    name: str
    marker_type: int
    position: tuple[float, float, float]
    scale: tuple[float, float, float]
    color: Color


SO101_TASK_ENV_SPECS: dict[str, tuple[MarkerSpec, ...]] = {
    "table_reach": (
        MarkerSpec("table_top", Marker.CUBE, (0.22, 0.0, 0.19), (0.36, 0.48, 0.04), Color(0.62, 0.52, 0.42)),
        MarkerSpec("reach_target", Marker.SPHERE, (0.25, 0.0, 0.255), (0.036, 0.036, 0.036), Color(0.1, 0.8, 0.3)),
    ),
    "pick_place": (
        MarkerSpec("table_top", Marker.CUBE, (0.22, 0.0, 0.19), (0.36, 0.48, 0.04), Color(0.62, 0.52, 0.42)),
        MarkerSpec("pickup_cube_start", Marker.CUBE, (0.22, -0.07, 0.225), (0.03, 0.03, 0.03), Color(0.88, 0.2, 0.2)),
        MarkerSpec("goal_pad", Marker.CYLINDER, (0.28, 0.09, 0.205), (0.06, 0.06, 0.006), Color(0.2, 0.45, 0.95)),
    ),
    "bin_sort": (
        MarkerSpec("table_top", Marker.CUBE, (0.22, 0.0, 0.19), (0.36, 0.48, 0.04), Color(0.62, 0.52, 0.42)),
        MarkerSpec("left_bin_base", Marker.CUBE, (0.27, -0.10, 0.215), (0.09, 0.09, 0.02), Color(0.82, 0.25, 0.25, 0.45)),
        MarkerSpec("right_bin_base", Marker.CUBE, (0.27, 0.10, 0.215), (0.09, 0.09, 0.02), Color(0.22, 0.45, 0.88, 0.45)),
        MarkerSpec("cube_yellow_start", Marker.CUBE, (0.19, -0.03, 0.225), (0.028, 0.028, 0.028), Color(0.95, 0.82, 0.12)),
        MarkerSpec("cube_teal_start", Marker.CUBE, (0.19, 0.05, 0.225), (0.028, 0.028, 0.028), Color(0.12, 0.78, 0.78)),
    ),
}


def markers_for_task_env(task_env: str, frame_id: str = "base_link") -> MarkerArray:
    array = MarkerArray()
    for index, spec in enumerate(SO101_TASK_ENV_SPECS.get(task_env, ())):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "task_env"
        marker.id = index
        marker.type = spec.marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = spec.position[0]
        marker.pose.position.y = spec.position[1]
        marker.pose.position.z = spec.position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = spec.scale[0]
        marker.scale.y = spec.scale[1]
        marker.scale.z = spec.scale[2]
        marker.color.r = spec.color.r
        marker.color.g = spec.color.g
        marker.color.b = spec.color.b
        marker.color.a = spec.color.a
        marker.lifetime = Duration(seconds=0).to_msg()
        array.markers.append(marker)
    return array


class TaskEnvMarkers(Node):
    def __init__(self) -> None:
        super().__init__("task_env_markers")
        self.declare_parameter("robot", "so101")
        self.declare_parameter("task_env", "default")
        robot = str(self.get_parameter("robot").value)
        task_env = str(self.get_parameter("task_env").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(MarkerArray, MARKER_TOPIC, qos)
        self._markers = MarkerArray()

        if robot == "so101" and task_env in SO101_TASK_ENV_SPECS:
            self._markers = markers_for_task_env(task_env)
            self.get_logger().info(
                f"Publishing {len(self._markers.markers)} task-env markers for so101/{task_env} on {MARKER_TOPIC}"
            )
        else:
            self.get_logger().info(
                f"No task-env markers configured for robot={robot!r} task_env={task_env!r}; topic stays empty"
            )

        self._publish_once()
        self._timer = self.create_timer(1.0, self._publish_once)

    def _publish_once(self) -> None:
        self._publisher.publish(self._markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskEnvMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
