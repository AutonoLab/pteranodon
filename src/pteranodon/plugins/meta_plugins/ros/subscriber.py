from rclpy.node import Node

PREFIX = "drone/mavsdk/pteranodon/"


class Subscriber(Node):
    """ROS subscriber node"""

    def __init__(self, name: str = "subscriber"):
        super().__init__(name)
        self.sub = None

    def subscribe(self, msg_type, name: str):
        """create subscriber"""
        self.sub = self.create_subscription(msg_type, name, self.listener_callback, 10)
        # self.sub

    def listener_callback(self, msg):
        """callback function for subscriber data"""
        self.get_logger().info(f"{msg.data}")
