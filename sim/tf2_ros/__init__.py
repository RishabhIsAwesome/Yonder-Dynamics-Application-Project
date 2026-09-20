"""
tf2_ros shim — minimal stub for Stretch Goal A.

Provides TransformBroadcaster so candidates can publish TF2 transforms
without a real ROS installation. Transforms are logged and stored for
the visualizer to optionally display.

Real tf2_ros usage (same API as this shim):

    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped

    class MyNode(Node):
        def __init__(self):
            super().__init__('my_node')
            self.tf_broadcaster = TransformBroadcaster(self)

        def publish_transform(self, x, y, yaw):
            t = TransformStamped()
            t.header_stamp = self.get_clock().now().nanoseconds / 1e9
            t.header_frame_id = 'odom'        # parent frame — odom, not base_link!
            t.child_frame_id  = 'base_link'   # child frame
            t.translation.x = x
            t.translation.y = y
            t.translation.z = 0.0
            # yaw → quaternion
            t.rotation.x = 0.0
            t.rotation.y = 0.0
            t.rotation.z = math.sin(yaw / 2)
            t.rotation.w = math.cos(yaw / 2)
            self.tf_broadcaster.sendTransform(t)

Common mistake: swapping parent/child frame order (base_link → odom instead
of odom → base_link). AI code generators do this frequently. The frame order
matters: child is described *relative to* parent.
"""

import time
import threading
from typing import Optional, List
from geometry_msgs.msg import TransformStamped

_transforms: List[TransformStamped] = []
_lock = threading.Lock()


def get_latest_transforms() -> List[TransformStamped]:
    """Used by the visualizer to read the most recent transform."""
    with _lock:
        return list(_transforms)


class TransformBroadcaster:
    """
    Broadcasts transforms to the TF2 tree.

    In a real ROS system this publishes to the /tf topic. In this shim
    it stores the transform in memory for the visualizer to read.
    """

    def __init__(self, node):
        self._node = node

    def sendTransform(self, transform: TransformStamped) -> None:
        with _lock:
            # Keep only recent transforms (one per child frame)
            global _transforms
            _transforms = [
                t for t in _transforms
                if t.child_frame_id != transform.child_frame_id
            ]
            _transforms.append(transform)
