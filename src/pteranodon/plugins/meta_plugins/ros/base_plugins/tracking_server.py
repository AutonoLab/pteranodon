from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
# from mavsdk.tracking_server import TrackPoint, TrackRectangle 
from pteranodon.plugins.base_plugins.tracking_server import TrackingServer

PREFIX = "drone/mavsdk/pteranodon/"

def ros_publish_tracking_point_command(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_tracking_rectangle_command(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_tracking_off_command(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_tracking_server_publishers(node: Node, tracking_server: TrackingServer):
    tracking_server.register_tracking_point_command_handler(
        handle_publisher(node, PREFIX + 'tracking_point_command', String, ros_publish_tracking_point_command)
    )
    tracking_server.register_tracking_rectangle_command_handler(
        handle_publisher(node, PREFIX + 'tracking_rectangle_command', String, ros_publish_tracking_rectangle_command)
    )
    tracking_server.register_tracking_off_command_handler(
        handle_publisher(node, PREFIX + 'tracking_off_command', String, ros_publish_tracking_off_command)
    )
