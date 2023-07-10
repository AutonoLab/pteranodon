from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Bool
from mavsdk.core import ConnectionState
from pteranodon.plugins.base_plugins.core import Core
from .handle_publisher import handle_publisher

PREFIX = "drone/mavsdk/pteranodon/"


def _ros_publish_connection_state(publisher: Publisher, data: ConnectionState) -> None:
    """
    Takes input of Publisher, and ConnectionState, consisting of:
        bool is_connected
    publishes Bool(is_connected) to ros topic

    """
    msg = Bool()
    msg.data = data.is_connected
    publisher.publish(msg)


def register_core_publishers(node: Node, core: Core) -> None:
    """Register handlers for core metrics"""
    core.register_connection_state_handler(
        handle_publisher(
            node, PREFIX + "connection_state", Bool, _ros_publish_connection_state
        )
    )
