from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class StaticTFPublisher(Node):
    """Publish static transforms on /tf_static."""

    def __init__(self):
        super().__init__("hpp_static_tf_publisher")
        self.broadcaster = StaticTransformBroadcaster(self)
        self.published = set()

    def publish(
        self,
        parent_frame,
        child_frame,
        xyz=(0.0, 0.0, 0.0),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    ):
        """
        Publish a static transform from parent_frame to child_frame with given translation and rotation.
        The transform is only published once per unique (parent_frame, child_frame) pair.

        args:
        parent_frame : str, ex: "world"
        child_frame  : str, ex: "box/base_link"
        xyz          : tuple of 3 floats, translation (x, y, z)
        quat_xyzw    : tuple of 4 floats, rotation as quaternion (x, y, z, w)

        """
        key = (parent_frame, child_frame)
        if key in self.published:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x = float(quat_xyzw[0])
        t.transform.rotation.y = float(quat_xyzw[1])
        t.transform.rotation.z = float(quat_xyzw[2])
        t.transform.rotation.w = float(quat_xyzw[3])
        self.broadcaster.sendTransform(t)
        self.published.add(key)
