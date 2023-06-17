from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
from mavsdk.camera import Mode#, CaptureInfo, Information, VideoStreamInfo, Status
from pteranodon.plugins.base_plugins.camera import Camera

PREFIX = "drone/mavsdk/pteranodon/"


def ros_publish_capture_info(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_information(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_video_stream_info(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_status(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_mode(publisher: Publisher, data: Mode) -> None:
    """
    Takes input of Publisher, and Mode, enum:
        UNKNOWN
        PHOTO
        VIDEO
    publishes String to ros topic
    """
    msg = String()
    msg.data = str(data)
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_camera_publishers(node: Node, camera: Camera):
    camera.register_capture_info_handler(
        handle_publisher(node, PREFIX + 'capture_info', String, ros_publish_capture_info)
    )
    camera.register_information_handler(
        handle_publisher(node, PREFIX + 'information', String, ros_publish_information)
    )
    camera.register_video_stream_info_handler(
        handle_publisher(node, PREFIX + 'video_stream_info', String, ros_publish_video_stream_info)
    )
    camera.register_status_handler(
        handle_publisher(node, PREFIX + 'status', String, ros_publish_status)
    )
    camera.register_mode_handler(
        handle_publisher(node, PREFIX + 'mode', String, ros_publish_mode)
    )