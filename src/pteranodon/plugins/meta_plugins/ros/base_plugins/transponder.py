from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

# from mavsdk.transponder import Transponder as TransponderType
from pteranodon.plugins.base_plugins.transponder import Transponder
from .handle_publisher import handle_publisher

PREFIX = "drone/mavsdk/pteranodon/"


def _ros_publish_transponder(publisher: Publisher, data) -> None:
    """No data in test"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)


def register_transponder_publishers(node: Node, transponder: Transponder) -> None:
    """Register handlers for transponder metrics"""
    transponder.register_transponder_handler(
        handle_publisher(node, PREFIX + "transponder", String, _ros_publish_transponder)
    )
