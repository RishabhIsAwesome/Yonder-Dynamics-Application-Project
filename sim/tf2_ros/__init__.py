"""
tf2_ros shim — minimal stub for Stretch Goals A and C.

Provides TransformBroadcaster so candidates can publish TF2 transforms
without a real ROS installation. The shim only *stores* the latest transform
for each child frame (get_latest_transforms()). There is no Buffer,
TransformListener or lookup_transform() here: if you attempt Stretch Goal C
you write that part yourself (composing map->odom with odom->base_link, and
interpolating between updates). Nothing draws the transforms, so check your work
by printing them.

Usage (field layout matches real ROS 2, except header.stamp is float seconds):

    import math
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped

    class MyNode(Node):
        def __init__(self):
            super().__init__('my_node')
            self.tf_broadcaster = TransformBroadcaster(self)

        def publish_transform(self, x, y, yaw):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'        # parent frame — odom, not base_link!
            t.child_frame_id  = 'base_link'   # child frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            # yaw -> quaternion (rotation about z only)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw / 2)
            t.transform.rotation.w = math.cos(yaw / 2)
            self.tf_broadcaster.sendTransform(t)

Common mistake: swapping parent/child frame order (base_link → odom instead
of odom → base_link). AI code generators do this frequently. The frame order
matters: child is described *relative to* parent.
"""

import threading
from typing import List, Union
from geometry_msgs.msg import TransformStamped

_transforms: List[TransformStamped] = []
_lock = threading.Lock()


def get_latest_transforms() -> List[TransformStamped]:
    """The most recent transform for each child frame that has been sent."""
    with _lock:
        return list(_transforms)


class TransformBroadcaster:
    """
    Broadcasts transforms to the TF2 tree.

    In a real ROS system this publishes to the /tf topic. In this shim
    it stores the transform in memory (see get_latest_transforms()).
    """

    def __init__(self, node):
        self._node = node

    def sendTransform(self, transform: Union[TransformStamped, List[TransformStamped]]) -> None:
        """Accepts one TransformStamped or a list of them, like real tf2_ros."""
        items = transform if isinstance(transform, (list, tuple)) else [transform]
        with _lock:
            # Keep only the latest transform per child frame
            global _transforms
            for item in items:
                _transforms = [
                    t for t in _transforms
                    if t.child_frame_id != item.child_frame_id
                ]
                _transforms.append(item)
