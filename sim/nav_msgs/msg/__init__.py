"""
nav_msgs shim — mirrors real ROS 2 nav_msgs field names exactly.

nav_msgs/Odometry field layout (matches real ROS 2 spec):
  header.stamp          float  (seconds since epoch)
  header.frame_id       str    (typically "odom")
  child_frame_id        str    (typically "base_link")
  pose.pose.position    Point  (x, y, z)
  pose.pose.orientation Quaternion (x, y, z, w)
  pose.covariance       list[36]
  twist.twist.linear    Vector3 (x=forward velocity, y, z)
  twist.twist.angular   Vector3 (z=yaw rate)
  twist.covariance      list[36]
"""

from dataclasses import dataclass, field

from std_msgs.msg import Header
from geometry_msgs.msg import (
    PoseWithCovariance,
    TwistWithCovariance,
    Pose,
    Point,
    Quaternion,
    Twist,
    Vector3,
)


@dataclass
class Odometry:
    header: Header = field(default_factory=Header)
    child_frame_id: str = "base_link"
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
    twist: TwistWithCovariance = field(default_factory=TwistWithCovariance)
