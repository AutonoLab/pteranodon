import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32MultiArray, Float64MultiArray, Float64, UInt8MultiArray, UInt64

PREFIX = "drone/mavsdk/pteranodon/"

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
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'actuator_control_target')
    # subscriber.subscribe(Bool, PREFIX + 'armed')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'attitude_angular_velocity_body')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'attitude_euler')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'attitude_quaternion')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'battery')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'camera_attitude_euler')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'camera_attitude_quaternion')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'distance_sensor')
    # subscriber.subscribe(String, PREFIX + 'flight_mode')
    # subscriber.subscribe(Float64MultiArray, PREFIX + 'ground_truth')
    # subscriber.subscribe(Float64, PREFIX + 'heading')
    # subscriber.subscribe(Bool, PREFIX + 'health_all_ok')
    # subscriber.subscribe(UInt8MultiArray, PREFIX + 'health')
    # subscriber.subscribe(Float64MultiArray, PREFIX + 'home')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'imu')
    # subscriber.subscribe(Bool, PREFIX + 'in_air')
    # subscriber.subscribe(String, PREFIX + 'landed_state')
    subscriber.subscribe(Float32MultiArray, PREFIX + 'odometry')
    # subscriber.subscribe(Float64MultiArray, PREFIX + 'position')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'position_velocity_ned')
    # subscriber.subscribe(Float64MultiArray, PREFIX + 'raw_gps')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'raw_imu')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'rc_status')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'scaled_imu')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'scaled_pressure')
    # subscriber.subscribe(String, PREFIX + 'status_text')
    # subscriber.subscribe(UInt64, PREFIX + 'unix_epoch_time')
    # subscriber.subscribe(Float32MultiArray, PREFIX + 'velocity_ned')
    # subscriber.subscribe(String, PREFIX + 'vtol_state')
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()