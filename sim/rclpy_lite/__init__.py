"""
rclpy_lite — lightweight ROS 2 simulator shim.

Mirrors the real rclpy API (Node, Publisher, Subscriber, QoS, spin) so
candidates can write ROS-style Python without installing ROS. To port code
to a real ROS 2 system, swap `rclpy_lite` → `rclpy` in imports.
"""

import logging

from ._bus import _Bus
from .node import Node
from .executors import MultiThreadedExecutor, SingleThreadedExecutor
from .qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    qos_profile_default,
)

_initialized = False


def init(*, args=None) -> None:
    global _initialized
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    _Bus.reset()
    _initialized = True


def ok() -> bool:
    return _initialized


def spin(node: Node) -> None:
    """Block until KeyboardInterrupt, running the node's timers."""
    import time
    try:
        while _initialized:
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


def spin_once(node: Node, *, timeout_sec: float = 0.1) -> None:
    import time
    time.sleep(timeout_sec)


def shutdown() -> None:
    global _initialized
    _initialized = False
    _Bus.reset()


def create_node(node_name: str) -> Node:
    return Node(node_name)
