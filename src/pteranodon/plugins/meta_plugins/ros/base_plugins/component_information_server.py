from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
from mavsdk.component_information_server import FloatParam
from pteranodon.plugins.base_plugins.component_information_server import ComponentInformationServer

PREFIX = "drone/mavsdk/pteranodon/"


def _ros_publish_float_param(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def _handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_component_info_server_publishers(node: Node, component_info_server: ComponentInformationServer) -> None:
    component_info_server.register_float_param_handler(
        _handle_publisher(node, PREFIX + 'float_param', _ros_publish_float_param)
    )