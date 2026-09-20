"""
geometry_msgs shim — mirrors real ROS 2 geometry_msgs field names exactly.
"""

from dataclasses import dataclass, field


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class PoseWithCovariance:
    pose: Pose = field(default_factory=Pose)
    covariance: list = field(default_factory=lambda: [0.0] * 36)


@dataclass
class Twist:
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class TwistWithCovariance:
    twist: Twist = field(default_factory=Twist)
    covariance: list = field(default_factory=lambda: [0.0] * 36)


@dataclass
class TransformStamped:
    """Used for TF2 stretch goal."""
    header_stamp: float = 0.0
    header_frame_id: str = ""        # parent frame
    child_frame_id: str = ""         # child frame
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)
