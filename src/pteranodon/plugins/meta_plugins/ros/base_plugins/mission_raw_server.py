from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
# from mavsdk.mission_raw_server import MissionItem
from pteranodon.plugins.base_plugins.mission_raw_server import MissionRawServer

PREFIX = "drone/mavsdk/pteranodon/"

def ros_publish_clear_all(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_current_item_changed(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def ros_publish_incoming_mission(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_mission_raw_server_publishers(node: Node, mission_raw_server: MissionRawServer):
    mission_raw_server.register_clear_all_handler(
        handle_publisher(node, PREFIX + 'clear_all', String, ros_publish_clear_all)
    )
    mission_raw_server.register_current_item_changed_handler(
        handle_publisher(node, PREFIX + 'current_item', String, ros_publish_current_item_changed)
    )
    mission_raw_server.register_incoming_mission_handler(
        handle_publisher(node, PREFIX + 'incoming_mission', String, ros_publish_incoming_mission)
    )
