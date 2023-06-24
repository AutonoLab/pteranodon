from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from typing import Callable
from std_msgs.msg import String
# from mavsdk.mission import 
from pteranodon.plugins.base_plugins.mission import Mission

PREFIX = "drone/mavsdk/pteranodon/"

def _ros_publish_incoming_mission(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), print(data))
    msg = String()
    msg.data = data
    publisher.publish(msg)

def _handle_publisher(node: Node, name: str, data_type, method: Callable) -> partial:
    """Create a publisher and pair it with a method to publish different mavsdk data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)

def register_mission_publishers(node: Node, mission: Mission) -> None:
    mission.register_incoming_mission_handler(
        _handle_publisher(node, PREFIX + 'incoming_mission', String, _ros_publish_incoming_mission)
    )
