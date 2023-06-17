from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import Bool
from mavsdk.core import ConnectionState
from pteranodon.plugins.base_plugins.core import Core

PREFIX = "drone/mavsdk/pteranodon/"

def ros_publish_connection_state(publisher: Publisher, data: ConnectionState) -> None:
    """
    Takes input of Publisher, and ConnectionState, consisting of:
        bool is_connected
    publishes Bool(is_connected) to ros topic

    """
    msg = Bool()
    msg.data = data.is_connected
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_core_publishers(node: Node, core: Core):
    core.register_connection_state_handler(
        handle_publisher(node, PREFIX + 'connection_state', Bool, ros_publish_connection_state)
    )
