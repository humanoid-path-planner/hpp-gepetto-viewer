from nav_msgs.msg import Path
from rclpy.node import Node


class NavigationPublisher(Node):
    """Publish navigation paths on /path (or /<namespace>/path)"""

    def __init__(self):
        super().__init__("pinnochio_navigation_publisher")
        self.publishers_map = {}
        self.last_paths = {}

    def _get_publisher(self, topic):
        if topic not in self.publishers_map:
            self.publishers_map[topic] = self.create_publisher(Path, topic, 10)
        return self.publishers_map[topic]

    def publish(self, path_msg: Path, topic_name: str | None = None):
        """Publish a path on a custom topic.
        args:
            path_msg   : nav_msgs.msg.Path, the path to publish
            topic_name : str or None, if None publish on /path, else on /<topic_name>/path
        """
        topic = topic_name if topic_name is not None else "path"
        publisher = self._get_publisher(topic)
        path_msg.header.stamp = self.get_clock().now().to_msg()
        publisher.publish(path_msg)
