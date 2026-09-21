"""
In-process topic message bus.

Connects publishers and subscribers, enforces QoS compatibility exactly as
real ROS 2 DDS does:
  - BEST_EFFORT publisher + RELIABLE subscriber  → no connection (with a warning)
  - BEST_EFFORT publisher + BEST_EFFORT subscriber → connected
  - RELIABLE publisher  + any subscriber          → connected

Also mirrors two behaviours of a real rclpy program:
  - Callbacks run one at a time (rclpy's default single-threaded executor), so
    your node's callbacks never run at the same time and you do not need locks.
  - An exception inside a callback is reported, not hidden. Unlike real rclpy
    it does not stop the program, so one bad message does not end the run.
"""

import logging
import threading
import traceback
from typing import Any, Callable, Dict, List, Optional

_log = logging.getLogger("rclpy")

# One lock for every subscriber and timer callback in the process.
_callback_lock = threading.RLock()

# Identical errors are printed in full once, then counted, so a bug that fires
# on every 50 Hz message does not bury the terminal.
_error_counts: Dict[tuple, int] = {}
_REPEAT_REPORT_AT = (10, 100, 1000, 10000, 100000)


def run_callback(where: str, fn: Callable, *args) -> None:
    """Run a subscriber/timer callback under the callback lock, reporting exceptions."""
    with _callback_lock:
        try:
            fn(*args)
        except Exception as exc:  # noqa: BLE001 - we want to report anything
            key = (where, type(exc).__name__, str(exc))
            n = _error_counts[key] = _error_counts.get(key, 0) + 1
            if n == 1:
                _log.error(
                    "Exception in %s (your code raised this; the run continues):\n%s",
                    where,
                    traceback.format_exc().rstrip(),
                )
            elif n in _REPEAT_REPORT_AT:
                _log.error(
                    "Same exception in %s has now happened %d times: %s: %s",
                    where, n, type(exc).__name__, exc,
                )


class _TopicEntry:
    def __init__(self, msg_type, qos):
        self.msg_type = msg_type
        self.qos = qos
        self.subscribers: List["_SubEntry"] = []
        self.lock = threading.Lock()

    def subs_snapshot(self) -> List["_SubEntry"]:
        with self.lock:
            return list(self.subscribers)


class _SubEntry:
    def __init__(self, callback: Callable, qos, node_name: str, handle):
        self.callback = callback
        self.qos = qos
        self.node_name = node_name
        self.handle = handle  # the _Subscription the node holds; .connected is kept up to date


class _Bus:
    """Singleton in-process message bus shared across all nodes."""

    _instance: Optional["_Bus"] = None
    _lock = threading.Lock()

    @classmethod
    def instance(cls) -> "_Bus":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset(cls) -> None:
        with cls._lock:
            cls._instance = None

    def __init__(self):
        self._topics: Dict[str, _TopicEntry] = {}
        self._lock = threading.RLock()

    # ------------------------------------------------------------------
    # Publisher side
    # ------------------------------------------------------------------

    def register_publisher(self, topic: str, msg_type, qos) -> None:
        with self._lock:
            if topic not in self._topics:
                self._topics[topic] = _TopicEntry(msg_type, qos)
            else:
                # Update QoS to this publisher's setting (last wins, realistic enough)
                self._topics[topic].qos = qos

    def publish(self, topic: str, msg: Any) -> None:
        with self._lock:
            entry = self._topics.get(topic)
            if entry is None:
                return
            subs = entry.subs_snapshot()

        for sub in subs:
            run_callback(f"subscription callback for '{topic}' ({sub.node_name})", sub.callback, msg)

    # ------------------------------------------------------------------
    # Subscriber side
    # ------------------------------------------------------------------

    def register_subscriber(
        self,
        topic: str,
        msg_type,
        callback: Callable,
        qos,
        node_name: str,
        handle=None,
    ) -> bool:
        """
        Returns True if connected, False on a QoS mismatch (a warning is logged).
        """
        with self._lock:
            if topic not in self._topics:
                # Publisher not yet registered — store the sub and connect later
                self._topics[topic] = _TopicEntry(msg_type, qos=None)

            entry = self._topics[topic]
            pub_qos = entry.qos

            if pub_qos is not None and not _qos_compatible(pub_qos, qos):
                _warn_incompatible(topic, node_name, pub_qos, qos)
                if handle is not None:
                    handle.connected = False
                return False

            if handle is not None:
                handle.connected = True
            with entry.lock:
                entry.subscribers.append(_SubEntry(callback, qos, node_name, handle))
            return True

    def late_connect_subscribers(self, topic: str, pub_qos) -> None:
        """
        Called when a publisher registers after subscribers already exist.
        Drops any subscribers whose QoS is incompatible.
        """
        with self._lock:
            entry = self._topics.get(topic)
            if entry is None:
                return
            compatible = []
            with entry.lock:
                for sub in entry.subscribers:
                    if _qos_compatible(pub_qos, sub.qos):
                        compatible.append(sub)
                    else:
                        _warn_incompatible(topic, sub.node_name, pub_qos, sub.qos)
                        if sub.handle is not None:
                            sub.handle.connected = False
                entry.subscribers = compatible


def _warn_incompatible(topic: str, node_name: str, pub_qos, sub_qos) -> None:
    _log.warning(
        "Subscription to '%s' in node '%s' has incompatible QoS with the publisher: "
        "publisher offers %s, subscriber requests %s. No messages will be delivered "
        "(policy: RELIABILITY_QOS_POLICY).",
        topic, node_name, pub_qos.reliability.name, sub_qos.reliability.name,
    )


def _qos_compatible(pub_qos, sub_qos) -> bool:
    """
    ROS 2 rule: subscriber reliability must be <= publisher reliability.
    RELIABLE > BEST_EFFORT, so:
      pub=BEST_EFFORT, sub=RELIABLE  → incompatible (sub wants more than pub provides)
      pub=RELIABLE,    sub=RELIABLE  → compatible
      pub=RELIABLE,    sub=BEST_EFFORT → compatible
      pub=BEST_EFFORT, sub=BEST_EFFORT → compatible
    """
    from .qos import ReliabilityPolicy

    if pub_qos.reliability == ReliabilityPolicy.BEST_EFFORT:
        return sub_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    return True
