from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

# from mavsdk.shell import
from pteranodon.plugins.base_plugins.shell import Shell
from .handle_publisher import handle_publisher

PREFIX = "drone/mavsdk/pteranodon/"


def _ros_publish_receive(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)


def register_shell_publishers(node: Node, shell: Shell) -> None:
    """Register handlers for shell metrics"""
    shell.register_receive_handler(
        handle_publisher(node, PREFIX + "receive", String, _ros_publish_receive)
    )
