"""
geometry_msgs shim — mirrors real ROS 2 geometry_msgs field names exactly.
"""

from dataclasses import dataclass, field

from std_msgs.msg import Header


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
class Transform:
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:
    """
    Used for the TF2 stretch goals. Same field layout as the real ROS 2 message:
      header.stamp / header.frame_id   the PARENT frame (e.g. "odom")
      child_frame_id                   the CHILD frame (e.g. "base_link")
      transform.translation / .rotation   the child's pose *relative to* the parent
    (Only difference: header.stamp is float seconds, not builtin_interfaces/Time.)
    """
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    transform: Transform = field(default_factory=Transform)
