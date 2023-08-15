# from rclpy.node import Node
# from rclpy.publisher import Publisher
# from std_msgs.msg import String

# # from mavsdk.component_information_server import FloatParam
# from pteranodon.plugins.base_plugins.component_information_server import (
#     ComponentInformationServer,
# )
# from ..prefix import PREFIX
# from .handle_publisher import handle_publisher


# def _ros_publish_float_param(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def register_component_info_server_publishers(
#     node: Node, component_info_server: ComponentInformationServer
# ) -> None:
#     """Register handlers for component info server metrics"""
#     component_info_server.register_float_param_handler(
#         handle_publisher(node, PREFIX + "float_param", String, _ros_publish_float_param)
#     )
