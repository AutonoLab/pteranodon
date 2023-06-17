from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
# from mavsdk.transponder import Transponder as TransponderType
from pteranodon.plugins.base_plugins.transponder import Transponder

PREFIX = "drone/mavsdk/pteranodon/"

def ros_publish_transponder(publisher: Publisher, data) -> None:
    """No data in test"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_transponder_publishers(node: Node, transponder: Transponder):
    transponder.register_transponder_handler(
        handle_publisher(node, PREFIX + 'transponder', String, ros_publish_transponder)
    )
