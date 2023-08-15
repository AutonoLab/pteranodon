import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Publisher(Node):
    """ROS publisher node"""

    def __init__(self, name: str = "publisher"):
        super().__init__(name)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.publisher_ = self.create_publisher(String, "message", 10)

    def timer_callback(self):
        """callback function for subscriber data"""
        msg = String()
        msg.data = "test publish"
        self.publisher_.publish(msg)
        # self.get_logger().info(f"{msg.data}")


def _main(args=None):
    """Test in main"""
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    _main()
