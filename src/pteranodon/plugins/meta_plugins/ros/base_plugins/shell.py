from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
# from mavsdk.shell import 
from pteranodon.plugins.base_plugins.shell import Shell

PREFIX = "drone/mavsdk/pteranodon/"

def ros_publish_receive(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_shell_publishers(node: Node, shell: Shell):
    shell.register_receive_handler(
        handle_publisher(node, PREFIX + 'receive', String, ros_publish_receive)
    )
