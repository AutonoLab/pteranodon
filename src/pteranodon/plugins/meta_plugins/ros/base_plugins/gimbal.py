from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
from mavsdk.gimbal import ControlMode
from pteranodon.plugins.base_plugins.gimbal import Gimbal

PREFIX = "drone/mavsdk/pteranodon/"

def ros_publish_control(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_gimbal_publishers(node: Node, gimbal: Gimbal):
    gimbal.register_control_handler(
        handle_publisher(node, PREFIX + 'control', String, ros_publish_control)
    )
