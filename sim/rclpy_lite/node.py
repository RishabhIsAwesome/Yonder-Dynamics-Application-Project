"""
Node shim — mirrors rclpy.node.Node.

Supports:
  create_publisher(msg_type, topic, qos)
  create_subscription(msg_type, topic, callback, qos)
  create_timer(period_sec, callback)
  get_logger()
  get_clock().now()
  destroy_node()

Known differences from real rclpy (see the README): message timestamps are plain
floats (seconds), not builtin_interfaces/Time, and callbacks are serialised by a
single lock instead of an executor.
"""

import threading
import time
import logging
from typing import Any, Callable

from simclock import now as _sim_now

from ._bus import _Bus, run_callback
from .qos import QoSProfile


class _Logger:
    def __init__(self, name: str):
        self._log = logging.getLogger(name)

    def info(self, msg: str) -> None:
        self._log.info(msg)

    def warn(self, msg: str) -> None:
        self._log.warning(msg)

    def warning(self, msg: str) -> None:
        self._log.warning(msg)

    def error(self, msg: str) -> None:
        self._log.error(msg)

    def debug(self, msg: str) -> None:
        self._log.debug(msg)


class _Time:
    """Result of get_clock().now(). Same attribute names as rclpy's Time."""

    def __init__(self, seconds: float):
        self.nanoseconds = int(seconds * 1e9)

    def seconds_nanoseconds(self):
        return divmod(self.nanoseconds, 1_000_000_000)

    def to_msg(self) -> float:
        """Real rclpy returns a builtin_interfaces/Time. Here a stamp is float seconds."""
        return self.nanoseconds / 1e9


class _Clock:
    def now(self) -> _Time:
        return _Time(_sim_now())


class _Publisher:
    def __init__(self, topic: str, msg_type, qos: QoSProfile, bus: _Bus):
        self._topic = topic
        self._msg_type = msg_type
        self._qos = qos
        self._bus = bus

    def publish(self, msg: Any) -> None:
        self._bus.publish(self._topic, msg)


class _Subscription:
    def __init__(self, topic: str, connected: bool = False):
        self.topic = topic
        self.connected = connected   # False if the QoS did not match the publisher


class _Timer:
    def __init__(self, period_sec: float, callback: Callable, node_name: str):
        self._period = period_sec
        self._callback = callback
        self._node_name = node_name
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name=f"{node_name}/timer", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        next_t = time.monotonic() + self._period
        while self._running:
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if not self._running:
                break
            run_callback(f"timer callback ({self._node_name})", self._callback)
            next_t += self._period
            if next_t < time.monotonic():
                next_t = time.monotonic() + self._period   # fell behind: don't burst

    def cancel(self) -> None:
        self._running = False


class Node:
    def __init__(self, node_name: str):
        self._name = node_name
        self._logger = _Logger(f"rclpy.{node_name}")
        self._clock = _Clock()
        self._bus = _Bus.instance()
        self._publishers: list = []
        self._subscriptions: list = []
        self._timers: list = []

    def get_name(self) -> str:
        return self._name

    def get_logger(self) -> _Logger:
        return self._logger

    def get_clock(self) -> _Clock:
        return self._clock

    def create_publisher(self, msg_type, topic: str, qos_profile) -> _Publisher:
        if isinstance(qos_profile, int):
            qos_profile = QoSProfile(depth=qos_profile)
        self._bus.register_publisher(topic, msg_type, qos_profile)
        self._bus.late_connect_subscribers(topic, qos_profile)
        pub = _Publisher(topic, msg_type, qos_profile, self._bus)
        self._publishers.append(pub)
        return pub

    def create_subscription(
        self, msg_type, topic: str, callback: Callable, qos_profile
    ) -> _Subscription:
        if isinstance(qos_profile, int):
            qos_profile = QoSProfile(depth=qos_profile)
        sub = _Subscription(topic)
        self._bus.register_subscriber(
            topic, msg_type, callback, qos_profile, self._name, handle=sub
        )
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, timer_period_sec: float, callback: Callable) -> _Timer:
        timer = _Timer(timer_period_sec, callback, self._name)
        self._timers.append(timer)
        return timer

    def destroy_node(self) -> None:
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
