from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
# from mavsdk.mission_raw import MissionProgress
from pteranodon.plugins.base_plugins.mission_raw import MissionRaw

PREFIX = "drone/mavsdk/pteranodon/"

def _ros_publish_mission_changed(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def _ros_publish_mission_progress(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

def _handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_mission_raw_publishers(node: Node, mission_raw: MissionRaw) -> None:
    mission_raw.register_mission_changed_handler(
        _handle_publisher(node, PREFIX + 'mission_changed', String, _ros_publish_mission_changed)
    )
    mission_raw.register_mission_progress_handler(
        _handle_publisher(node, PREFIX + 'mission_progress', String, _ros_publish_mission_progress)
    )