# from rclpy.node import Node
# from rclpy.publisher import Publisher
# from std_msgs.msg import String

# # from mavsdk.camera_server import TakePhotoFeedback
# from pteranodon.plugins.base_plugins.camera_server import CameraServer
# from ..prefix import PREFIX
# from .handle_publisher import handle_publisher


# def _ros_publish_take_photo(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def register_camera_server_publishers(node: Node, camera_server: CameraServer) -> None:
#     """Register handlers for camera server metrics"""
#     camera_server.register_take_photo_handler(
#         handle_publisher(node, PREFIX + "take_photo", String, _ros_publish_take_photo)
#     )
