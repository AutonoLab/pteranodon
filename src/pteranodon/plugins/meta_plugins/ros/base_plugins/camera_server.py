from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
# from mavsdk.camera_server import TakePhotoFeedback
from pteranodon.plugins.base_plugins.camera_server import CameraServer

PREFIX = "drone/mavsdk/pteranodon/"


def ros_publish_take_photo(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_camera_server_publishers(node: Node, camera_server: CameraServer) -> partial:
    camera_server.register_take_photo_handler(
        handle_publisher(node, PREFIX + 'take_photo', ros_publish_take_photo)
    )