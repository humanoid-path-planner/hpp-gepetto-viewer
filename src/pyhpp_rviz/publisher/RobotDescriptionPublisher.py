from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    """Publish robot_description as a topic (per-namespace)."""

    def __init__(self):
        super().__init__("pinnochio_robot_description_publisher")
        self.publishers_map = {}

    def _get_publisher(self, namespace):
        topic = f"{namespace}/robot_description" if namespace else "robot_description"
        if topic not in self.publishers_map:
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = ReliabilityPolicy.RELIABLE
            self.publishers_map[topic] = self.create_publisher(String, topic, qos)
        return self.publishers_map[topic]

    def publish(self, namespace, description):
        self._get_publisher(namespace).publish(String(data=description))
