"""
Executor shim — mirrors rclpy.executors.

In the simulator, nodes are already spinning via their own timer threads.
The executor here just keeps the main thread alive and provides the
add_node() / spin() interface that real rclpy code expects.
"""

import threading
import time
from typing import List


class _BaseExecutor:
    def __init__(self):
        self._nodes: List = []
        self._running = False

    def add_node(self, node) -> None:
        self._nodes.append(node)

    def spin(self) -> None:
        self._running = True
        try:
            while self._running:
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        time.sleep(timeout_sec)

    def shutdown(self) -> None:
        self._running = False


class SingleThreadedExecutor(_BaseExecutor):
    pass


class MultiThreadedExecutor(_BaseExecutor):
    def __init__(self, num_threads: int = 4):
        super().__init__()
        self._num_threads = num_threads
