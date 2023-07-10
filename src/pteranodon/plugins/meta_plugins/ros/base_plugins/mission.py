from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

# from mavsdk.mission import
from pteranodon.plugins.base_plugins.mission import Mission
from .handle_publisher import handle_publisher

PREFIX = "drone/mavsdk/pteranodon/"


def _ros_publish_incoming_mission(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), print(data))
    msg = String()
    msg.data = data
    publisher.publish(msg)


def register_mission_publishers(node: Node, mission: Mission) -> None:
    """Register handlers for mission metrics"""
    mission.register_incoming_mission_handler(
        handle_publisher(
            node, PREFIX + "incoming_mission", String, _ros_publish_incoming_mission
        )
    )
