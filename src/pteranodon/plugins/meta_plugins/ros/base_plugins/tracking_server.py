# from rclpy.node import Node
# from rclpy.publisher import Publisher
# from std_msgs.msg import String

# # from mavsdk.tracking_server import TrackPoint, TrackRectangle
# from pteranodon.plugins.base_plugins.tracking_server import TrackingServer
# from ..prefix import PREFIX
# from .handle_publisher import handle_publisher


# def _ros_publish_tracking_point_command(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_tracking_rectangle_command(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_tracking_off_command(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def register_tracking_server_publishers(
#     node: Node, tracking_server: TrackingServer
# ) -> None:
#     """Register handlers for tracking server metrics"""
#     tracking_server.register_tracking_point_command_handler(
#         handle_publisher(
#             node,
#             PREFIX + "tracking_point_command",
#             String,
#             _ros_publish_tracking_point_command,
#         )
#     )
#     tracking_server.register_tracking_rectangle_command_handler(
#         handle_publisher(
#             node,
#             PREFIX + "tracking_rectangle_command",
#             String,
#             _ros_publish_tracking_rectangle_command,
#         )
#     )
#     tracking_server.register_tracking_off_command_handler(
#         handle_publisher(
#             node,
#             PREFIX + "tracking_off_command",
#             String,
#             _ros_publish_tracking_off_command,
#         )
#     )
