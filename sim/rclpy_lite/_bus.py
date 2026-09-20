"""
In-process topic message bus.

Connects publishers and subscribers, enforces QoS compatibility exactly as
real ROS 2 DDS does:
  - BEST_EFFORT publisher + RELIABLE subscriber  → silent no-connection
  - BEST_EFFORT publisher + BEST_EFFORT subscriber → connected
  - RELIABLE publisher  + any subscriber          → connected
"""

import threading
import time
from typing import Any, Callable, Dict, List, Optional


class _TopicEntry:
    def __init__(self, msg_type, qos):
        self.msg_type = msg_type
        self.qos = qos
        self.subscribers: List["_SubEntry"] = []
        self.lock = threading.Lock()


class _SubEntry:
    def __init__(self, callback: Callable, qos, node_name: str):
        self.callback = callback
        self.qos = qos
        self.node_name = node_name


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
        self._logger_fn: Callable[[str], None] = lambda msg: None

    def set_logger(self, fn: Callable[[str], None]) -> None:
        self._logger_fn = fn

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
            subs = list(entry.subs_snapshot())

        for sub in subs:
            try:
                sub.callback(msg)
            except Exception as exc:
                self._logger_fn(f"[bus] callback error on {topic}: {exc}")

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
    ) -> bool:
        """
        Returns True if connected, False if QoS mismatch (silent in log only).
        """
        with self._lock:
            if topic not in self._topics:
                # Publisher not yet registered — store the sub and connect later
                self._topics[topic] = _TopicEntry(msg_type, qos=None)

            entry = self._topics[topic]
            pub_qos = entry.qos

            if pub_qos is not None and not _qos_compatible(pub_qos, qos):
                self._logger_fn(
                    f"[bus] QoS mismatch on '{topic}': "
                    f"publisher={pub_qos.reliability.name}, "
                    f"subscriber={qos.reliability.name} — "
                    f"no connection (this is the same silent failure as real ROS 2)"
                )
                return False

            entry.subscribers.append(_SubEntry(callback, qos, node_name))
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
            for sub in entry.subscribers:
                if _qos_compatible(pub_qos, sub.qos):
                    compatible.append(sub)
                else:
                    self._logger_fn(
                        f"[bus] QoS mismatch on '{topic}': "
                        f"publisher={pub_qos.reliability.name}, "
                        f"subscriber={sub.qos.reliability.name} — "
                        f"no connection"
                    )
            entry.subscribers = compatible


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


# Patch the _TopicEntry to expose a snapshot method
def _subs_snapshot(self):
    with self.lock:
        return list(self.subscribers)


_TopicEntry.subs_snapshot = _subs_snapshot
