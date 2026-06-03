"""ROS2 bridge — feed live data into the fm_tui app.

A background rclpy node subscribes to ``/rosout`` and reports node and topic
lists. It runs its executor on a daemon thread so it never blocks Textual's
event loop; log messages reach the UI through a caller-supplied callback, which
the app routes back onto its own thread via ``call_from_thread``.
"""

from __future__ import annotations

from threading import Thread
from typing import Callable

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

# rcl_interfaces/msg/Log severity levels -> palette severity tokens.
# FATAL folds into error: the palette tops out at error.
_SEVERITY_BY_LEVEL = {
    Log.DEBUG: "debug",
    Log.INFO: "info",
    Log.WARN: "warn",
    Log.ERROR: "error",
    Log.FATAL: "error",
}


def severity_for(level: int) -> str:
    """Map a ``/rosout`` log level to a palette severity token."""
    return _SEVERITY_BY_LEVEL.get(level, "info")


class RosBridge:
    """Owns the rclpy node and streams /rosout, nodes, and topics to a callback."""

    def __init__(self, on_log: Callable[[str, str], None]) -> None:
        self._on_log = on_log
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._thread: Thread | None = None

    def start(self) -> None:
        """Initialise rclpy, subscribe to /rosout, and spin on a daemon thread."""
        rclpy.init()
        self._node = Node("fm_tui_monitor")
        self._node.create_subscription(Log, "/rosout", self._handle_log, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _handle_log(self, msg: Log) -> None:
        self._on_log(severity_for(msg.level), f"[{msg.name}] {msg.msg}")

    def nodes(self) -> list[str]:
        """Fully-qualified names of nodes currently visible on the graph."""
        if self._node is None:
            return []
        return sorted(
            f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            for name, namespace in self._node.get_node_names_and_namespaces()
        )

    def topics(self) -> list[str]:
        """Names of topics currently advertised on the graph."""
        if self._node is None:
            return []
        return sorted(name for name, _types in self._node.get_topic_names_and_types())

    def shutdown(self) -> None:
        """Stop the executor, destroy the node, and shut rclpy down."""
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
