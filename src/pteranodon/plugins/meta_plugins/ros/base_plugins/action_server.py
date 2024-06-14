# from rclpy.node import Node
# from rclpy.publisher import Publisher
# from mavsdk.action_server import ArmDisarm
# from std_msgs.msg import String, UInt8MultiArray
# from pteranodon.plugins.base_plugins.action_server import ActionServer
# from ..prefix import PREFIX
# from .handle_publisher import handle_publisher


# def _ros_publish_arm_disarm(publisher: Publisher, data: ArmDisarm) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = UInt8MultiArray()
#     msg.data = [data.arm, data.force]
#     publisher.publish(msg)


# def _ros_publish_flight_mode_change(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_land(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_reboot(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_shutdown(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_takeoff(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_terminate(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def register_action_server_publishers(node: Node, action: ActionServer) -> None:
#     """Register handlers for action server metrics"""
#     action.register_arm_disarm_handler(
#         handle_publisher(
#             node, PREFIX + "arm_disarm", UInt8MultiArray, _ros_publish_arm_disarm
#         )
#     )
#     action.register_flight_mode_change_handler(
#         handle_publisher(
#             node, PREFIX + "flight_mode_change", String, _ros_publish_flight_mode_change
#         )
#     )
#     action.register_land_handler(
#         handle_publisher(node, PREFIX + "land", String, _ros_publish_land)
#     )
#     action.register_reboot_handler(
#         handle_publisher(node, PREFIX + "reboot", String, _ros_publish_reboot)
#     )
#     action.register_shutdown_handler(
#         handle_publisher(node, PREFIX + "shutdown", String, _ros_publish_shutdown)
#     )
#     action.register_takeoff_handler(
#         handle_publisher(node, PREFIX + "takeoff", String, _ros_publish_takeoff)
#     )
#     action.register_terminate_handler(
#         handle_publisher(node, PREFIX + "terminate", String, _ros_publish_terminate)
#     )
