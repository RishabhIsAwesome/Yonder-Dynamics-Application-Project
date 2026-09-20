"""
QoS shim — mirrors rclpy.qos exactly so import statements work unchanged.
"""

from enum import Enum, auto


class ReliabilityPolicy(Enum):
    RELIABLE = auto()
    BEST_EFFORT = auto()


class DurabilityPolicy(Enum):
    VOLATILE = auto()
    TRANSIENT_LOCAL = auto()


class HistoryPolicy(Enum):
    KEEP_LAST = auto()
    KEEP_ALL = auto()


class QoSProfile:
    def __init__(
        self,
        *,
        depth: int = 10,
        reliability: ReliabilityPolicy = ReliabilityPolicy.RELIABLE,
        durability: DurabilityPolicy = DurabilityPolicy.VOLATILE,
        history: HistoryPolicy = HistoryPolicy.KEEP_LAST,
    ):
        self.depth = depth
        self.reliability = reliability
        self.durability = durability
        self.history = history

    def __repr__(self) -> str:
        return (
            f"QoSProfile(reliability={self.reliability.name}, "
            f"depth={self.depth})"
        )


# Pre-built profiles matching real rclpy constants
qos_profile_sensor_data = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)

qos_profile_default = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)
