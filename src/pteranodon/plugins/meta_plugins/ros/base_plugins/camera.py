from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from mavsdk.camera import Mode  # , CaptureInfo, Information, VideoStreamInfo, Status
from pteranodon.plugins.base_plugins.camera import Camera
from .handle_publisher import handle_publisher

PREFIX = "drone/mavsdk/pteranodon/"


# def _ros_publish_capture_info(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_information(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_video_stream_info(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_status(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


def _ros_publish_mode(publisher: Publisher, data: Mode) -> None:
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


def register_camera_publishers(node: Node, camera: Camera) -> None:
    """Register handlers for camera metrics"""
    # camera.register_capture_info_handler(
    #     handle_publisher(
    #         node, PREFIX + "capture_info", String, _ros_publish_capture_info
    #     )
    # )
    # camera.register_information_handler(
    #     handle_publisher(node, PREFIX + "information", String, _ros_publish_information)
    # )
    # camera.register_video_stream_info_handler(
    #     handle_publisher(
    #         node, PREFIX + "video_stream_info", String, _ros_publish_video_stream_info
    #     )
    # )
    # camera.register_status_handler(
    #     handle_publisher(node, PREFIX + "status", String, _ros_publish_status)
    # )
    camera.register_mode_handler(
        handle_publisher(node, PREFIX + "mode", String, _ros_publish_mode)
    )
