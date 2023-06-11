import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32MultiArray, Float64MultiArray, Float64, UInt8MultiArray, UInt64


class Subscriber(Node):

    def __init__(self, name:str="subscriber"):
        super().__init__(name)
        
    def subscribe(self, type, name: str):
        self.subscription = self.create_subscription(type, name, self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"{msg.data}")
        #print(type(msg), "\n", type(msg.data[1]), "\n", msg.data[1], "\n")


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()
    # subscriber.subscribe(String, 'actuator_control_target')
    # subscriber.subscribe(Bool, 'armed')
    # subscriber.subscribe(Float32MultiArray, 'attitude_angular_velocity_body')
    # subscriber.subscribe(Float32MultiArray, 'attitude_euler')
    # subscriber.subscribe(Float32MultiArray, 'attitude_quaternion')
    # subscriber.subscribe(Float32MultiArray, 'battery')
    # subscriber.subscribe(Float32MultiArray, 'camera_attitude_euler')
    # subscriber.subscribe(Float32MultiArray, 'camera_attitude_quaternion')
    # subscriber.subscribe(Float32MultiArray, 'distance_sensor')
    # subscriber.subscribe(String, 'flight_mode')
    # subscriber.subscribe(Float64MultiArray, 'ground_truth')
    # subscriber.subscribe(Float64, 'heading')
    # subscriber.subscribe(Bool, 'health_all_ok')
    # subscriber.subscribe(UInt8MultiArray, 'health')
    # subscriber.subscribe(Float64MultiArray, 'home')
    # subscriber.subscribe(Float32MultiArray, 'imu')
    # subscriber.subscribe(Bool, 'in_air')
    # subscriber.subscribe(String, 'landed_state')
    # subscriber.subscribe(String, 'odometry')
    # subscriber.subscribe(Float64MultiArray, 'position')
    # subscriber.subscribe(Float32MultiArray, 'position_velocity_ned')
    # subscriber.subscribe(Float64MultiArray, 'raw_gps')
    # subscriber.subscribe(Float32MultiArray, 'raw_imu')
    # subscriber.subscribe(Float32MultiArray, 'rc_status')
    # subscriber.subscribe(Float32MultiArray, 'scaled_imu')
    # subscriber.subscribe(Float32MultiArray, 'scaled_pressure')
    # subscriber.subscribe(String, 'status_text')
    # subscriber.subscribe(UInt64, 'unix_epoch_time')
    # subscriber.subscribe(Float32MultiArray, 'velocity_ned')
    # subscriber.subscribe(String, 'vtol_state')
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()