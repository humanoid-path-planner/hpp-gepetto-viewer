from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TransformStampedPublisher(Node):
    """
    Publish transform stamped messages on /tf via TransformBroadcaster.
    For each frame, broadcast a TF from parent (ex:"world") to  Child frame (ex:"<namespace>/base_link").
    """

    def __init__(self):
        super().__init__("hpp_tf_broadcaster")
        self.broadcaster = TransformBroadcaster(self)
        self.last_transforms = {}
