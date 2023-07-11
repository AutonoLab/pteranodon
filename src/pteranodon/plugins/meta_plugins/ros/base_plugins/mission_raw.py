# from rclpy.node import Node
# from rclpy.publisher import Publisher
# from std_msgs.msg import String

# # from mavsdk.mission_raw import MissionProgress
# from pteranodon.plugins.base_plugins.mission_raw import MissionRaw
# from .handle_publisher import handle_publisher

# PREFIX = "drone/mavsdk/pteranodon/"


# def _ros_publish_mission_changed(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def _ros_publish_mission_progress(publisher: Publisher, data) -> None:
#     """No test data"""
#     print(type(data), data)
#     msg = String()
#     msg.data = data
#     publisher.publish(msg)


# def register_mission_raw_publishers(node: Node, mission_raw: MissionRaw) -> None:
#     """Register handlers for mission raw metrics"""
#     mission_raw.register_mission_changed_handler(
#         handle_publisher(
#             node, PREFIX + "mission_changed", String, _ros_publish_mission_changed
#         )
#     )
#     mission_raw.register_mission_progress_handler(
#         handle_publisher(
#             node, PREFIX + "mission_progress", String, _ros_publish_mission_progress
#         )
#     )
