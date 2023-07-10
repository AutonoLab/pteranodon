from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

# from mavsdk.mission_raw_server import MissionItem
from pteranodon.plugins.base_plugins.mission_raw_server import MissionRawServer
from .handle_publisher import handle_publisher

PREFIX = "drone/mavsdk/pteranodon/"


def _ros_publish_clear_all(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)


def _ros_publish_current_item_changed(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)


def _ros_publish_incoming_mission(publisher: Publisher, data) -> None:
    """No test data"""
    print(type(data), data)
    msg = String()
    msg.data = data
    publisher.publish(msg)


def register_mission_raw_server_publishers(
    node: Node, mission_raw_server: MissionRawServer
) -> None:
    """Register handlers for mission raw server metrics"""
    mission_raw_server.register_clear_all_handler(
        handle_publisher(node, PREFIX + "clear_all", String, _ros_publish_clear_all)
    )
    mission_raw_server.register_current_item_changed_handler(
        handle_publisher(
            node, PREFIX + "current_item", String, _ros_publish_current_item_changed
        )
    )
    mission_raw_server.register_incoming_mission_handler(
        handle_publisher(
            node, PREFIX + "incoming_mission", String, _ros_publish_incoming_mission
        )
    )
