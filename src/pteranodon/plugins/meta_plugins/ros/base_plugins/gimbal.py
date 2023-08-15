# from rclpy.node import Node
# from rclpy.publisher import Publisher
# from std_msgs.msg import String

# # from mavsdk.gimbal import ControlMode, ControlStatus
# from pteranodon.plugins.base_plugins.gimbal import Gimbal
# from ..prefix import PREFIX
# from .handle_publisher import handle_publisher


# def _ros_publish_control(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def register_gimbal_publishers(node: Node, gimbal: Gimbal) -> None:
#     """Register handlers for gimbal metrics"""
#     gimbal.register_control_handler(
#         handle_publisher(node, PREFIX + "control", String, _ros_publish_control)
#     )
