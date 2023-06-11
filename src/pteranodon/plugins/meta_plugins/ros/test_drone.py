from pteranodon import SimpleDrone, AbstractDrone
import time
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from functools import partial
from mavsdk.telemetry import (
    AngularVelocityBody, EulerAngle, Quaternion, 
    Battery, DistanceSensor, FixedwingMetrics, 
    FlightMode, GpsInfo, GroundTruth, Heading, 
    Health, Position, Imu, Odometry, LandedState, 
    PositionVelocityNed, RawGps, RcStatus, 
    ScaledPressure, StatusText, VelocityNed, VtolState
)
from std_msgs.msg import String, Bool, Float32MultiArray, Float64MultiArray, Float64, UInt8MultiArray, MultiArrayDimension, UInt64


# bool
def ros_publish_bool(publisher: Publisher, data: bool):
    msg = Bool()
    msg.data = data
    publisher.publish(msg)

# str or unhandled type conversion
def ros_publish_string(publisher: Publisher, data) -> None:
    msg = String()
    msg.data = str(data)
    publisher.publish(msg)
    # ros_node.get_logger().info(f"{msg.data}")

# AngularVelocityBody
def ros_publish_velocity_body(publisher: Publisher, data: AngularVelocityBody) -> None:
    """Publish mavsdk AngularVelocityBody on ros publisher"""
    msg = Float32MultiArray()
    msg.data = [data.roll_rad_s, data.pitch_rad_s, data.yaw_rad_s]
    publisher.publish(msg)

# EulerAngle
def ros_publish_euler(publisher: Publisher, data: EulerAngle) -> None:
    msg = Float32MultiArray()
    msg.data = [data.roll_deg, data.pitch_deg, data.yaw_deg]
    publisher.publish(msg)

# Quaternion
def ros_publish_quaternion(publisher: Publisher, data: Quaternion) -> None:
    msg = Float32MultiArray()
    msg.data = [data.w, data.x, data.y, data.z]
    publisher.publish(msg)

# Battery
def ros_publish_battery(publisher: Publisher, data: Battery) -> None:
    msg = Float32MultiArray()
    # could add float(data.id) to msg data if necessary
    msg.data = [data.voltage_v, data.remaining_percent]
    publisher.publish(msg)

# DistanceSensor
def ros_publish_distance(publisher: Publisher, data: DistanceSensor) -> None:
    msg = Float32MultiArray()
    msg.data = [data.minimum_distance_m, data.maximum_distance_m, data.current_distance_m]
    publisher.publish(msg)

# FixedwingMetrics <-- probably not necessary for drone use
def ros_publish_fixedwing(publisher: Publisher, data: FixedwingMetrics) -> None:
    msg = Float32MultiArray()
    msg.data = [data.airspeed_m_s, data.throttle_percentage, data.climb_rate_m_s]
    publisher.publish(msg)

# FlightMode (ENUM)
def ros_publish_flight_mode(publisher: Publisher, data: FlightMode) -> None:
    msg = String()
    msg.data = data
    publisher.publish(msg)

# GPSInfo
def ros_publish_gps_info(publisher: Publisher, data: GpsInfo) -> None:
    msg = Float32MultiArray()
    msg.data = [data.num_satellites, data.fix_type]
    publisher.publish(msg)

# GroundTruth
def ros_publish_ground_truth(publisher: Publisher, data:GroundTruth) -> None:
    msg = Float64MultiArray()
    msg.data = [data.latitude_deg, data.longitude_deg, data.absolute_altitude_m]
    publisher.publish(msg)

# Heading
def ros_publish_heading(publisher: Publisher, data: Heading) -> None:
    msg = Float64()
    msg.data = data.heading_deg
    publisher.publish(msg)

# Health
def ros_publish_health(publisher: Publisher, data: Health) -> None:
    msg = UInt8MultiArray()
    msg.data = [
        data.is_armable,
        data.is_gyrometer_calibration_ok, 
        data.is_accelerometer_calibration_ok, 
        data.is_magnetometer_calibration_ok, 
        data.is_local_position_ok, 
        data.is_global_position_ok,
        data.is_home_position_ok
        ]
    publisher.publish(msg)

# Position
def ros_publish_position(publisher: Publisher, data: Position) -> None:
    msg = Float64MultiArray()
    msg.data = [data.latitude_deg, data.longitude_deg, data.absolute_altitude_m, data.relative_altitude_m]
    publisher.publish(msg)

# Imu
def ros_publish_imu(publisher: Publisher, data: Imu) -> None:
    msg = Float32MultiArray()
    msg.layout.dim = [MultiArrayDimension(label="rows", size=4), MultiArrayDimension(label="cols", size=3)]
    msg.data = [data.acceleration_frd.forward_m_s2, data.acceleration_frd.right_m_s2, data.acceleration_frd.down_m_s2, 
                data.angular_velocity_frd.forward_rad_s, data.angular_velocity_frd.right_rad_s, data.angular_velocity_frd.down_rad_s, 
                data.magnetic_field_frd.forward_gauss, data.magnetic_field_frd.right_gauss, data.magnetic_field_frd.down_gauss, 
                data.temperature_degc, float(data.timestamp_us)]
    publisher.publish(msg)

# LandedState (ENUM)
def ros_publish_landed_state(publisher: Publisher, data: LandedState) -> None:
    msg = String()
    msg.data = str(data)
    publisher.publish(msg)

# Odometry
def ros_publish_odometry(publisher: Publisher, data: Odometry) -> None:
    # print(data)
    msg = String()
    msg.data = data
    publisher.publish(msg)

# PositionVelocityNED
def ros_publish_position_velocity_ned(publisher: Publisher, data: PositionVelocityNed) -> None:
    msg = Float32MultiArray()
    msg.layout.dim = [MultiArrayDimension(label="rows", size=2), MultiArrayDimension(label="cols", size=3)]
    msg.data = [data.position.north_m, 
                data.position.east_m, 
                data.position.down_m, 
                data.velocity.north_m_s, 
                data.velocity.east_m_s, 
                data.velocity.down_m_s]
    publisher.publish(msg)

# RawGPS
def ros_publish_raw_gps(publisher: Publisher, data: RawGps) -> None:
    msg = Float64MultiArray()
    msg.data = [float(data.timestamp_us), 
                data.latitude_deg, 
                data.longitude_deg, 
                data.absolute_altitude_m, 
                data.hdop, 
                data.vdop, 
                data.velocity_m_s, 
                data.cog_deg, 
                data.altitude_ellipsoid_m, 
                data.horizontal_uncertainty_m, 
                data.vertical_uncertainty_m,
                data.velocity_uncertainty_m_s, 
                data.heading_uncertainty_deg, 
                data.yaw_deg]
    publisher.publish(msg)

# RCStatus
def ros_publish_rc_status(publisher: Publisher, data: RcStatus) -> None:
    msg = Float32MultiArray()
    msg.data = [float(data.was_available_once), float(data.is_available), data.signal_strength_percent]
    publisher.publish(msg)

# ScaledPressure
def ros_publish_scaled_pressure(publisher: Publisher, data: ScaledPressure) -> None:
    msg = Float32MultiArray()
    msg.data = [float(data.timestamp_us), 
                data.absolute_pressure_hpa, 
                data.differential_pressure_hpa, 
                data.temperature_deg, 
                data.differential_pressure_temperature_deg]
    publisher.publish(msg)

# StatusText
def ros_publish_status_text(publisher: Publisher, data: StatusText) -> None:
    msg = String()
    msg.data = str(str(data.type) + ": " + str(data.text))
    publisher.publish(msg)

# UnixEpochTime
def ros_publish_unix_epoch_time(publisher: Publisher, data: int) -> None:
    msg = UInt64()
    msg.data = data
    publisher.publish(msg)

# VelocityNed
def ros_publish_velocity_ned(publisher: Publisher, data: VelocityNed) -> None:
    msg = Float32MultiArray()
    msg.data = [data.north_m_s, data.east_m_s, data.down_m_s]
    publisher.publish(msg)

# VtolState (ENUM) <-- also probably not necessary for our drone
def ros_publish_vtol_state(publisher: Publisher, data: VtolState) -> None:
    print(data)
    msg = String()
    msg.data = str(data)
    publisher.publish(msg)


def handle_publisher(node: Node, name: str, data_type, method) -> partial:
    """Create a publisher and pair it with a method to publish different data types"""
    publisher = node.create_publisher(data_type, name, 10)
    return partial(method, publisher)


def register_telemetry_ros(node: Node, drone: AbstractDrone) -> None:
    drone.telemetry.register_armed_handler(
        handle_publisher(node, 'armed', Bool, ros_publish_bool)
    )
    drone.telemetry.register_actuator_control_target_handler(
        handle_publisher(node, 'actuator_control_target', String, ros_publish_string)
    )
    drone.telemetry.register_attitude_angular_velocity_body_handler(
        handle_publisher(node, 'attitude_angular_velocity_body', Float32MultiArray, ros_publish_velocity_body)
    )
    drone.telemetry.register_attitude_euler_handler(
        handle_publisher(node, 'attitude_euler', Float32MultiArray, ros_publish_euler)
    )
    drone.telemetry.register_attitude_quaternion_handler(
        handle_publisher(node, 'attitude_quaternion', Float32MultiArray, ros_publish_quaternion)
    )
    drone.telemetry.register_battery_handler(
        handle_publisher(node, 'battery', Float32MultiArray, ros_publish_battery)
    )
    drone.telemetry.register_camera_attitude_euler_handler(
        handle_publisher(node, 'camera_attitude_euler', Float32MultiArray, ros_publish_euler)
    )
    drone.telemetry.register_camera_attitude_quaternion_handler(
        handle_publisher(node, 'camera_attitude_quaternion', Float32MultiArray, ros_publish_quaternion)
    )
    drone.telemetry.register_distance_sensor_handler(
        handle_publisher(node, 'distance_sensor', Float32MultiArray, ros_publish_distance)
    )
    drone.telemetry.register_fixedwing_metrics_handler(
        handle_publisher(node, 'fixedwing_metrics', Float32MultiArray, ros_publish_fixedwing)
    )
    drone.telemetry.register_flight_mode_handler(
        handle_publisher(node, 'flight_mode', String, ros_publish_flight_mode)
    )
    drone.telemetry.register_gps_info_handler(
        handle_publisher(node, 'gps_info', Float32MultiArray, ros_publish_gps_info)
    )
    drone.telemetry.register_ground_truth_handler(
        handle_publisher(node, 'ground_truth', Float64MultiArray, ros_publish_ground_truth)
    )
    drone.telemetry.register_heading_handler(
        handle_publisher(node, 'heading', Float64, ros_publish_heading)
    )
    drone.telemetry.register_health_all_ok_handler(
        handle_publisher(node, 'health_all_ok', Bool, ros_publish_bool)
    )
    drone.telemetry.register_health_handler(
        handle_publisher(node, 'health', UInt8MultiArray, ros_publish_health)
    )
    drone.telemetry.register_home_handler(
        handle_publisher(node, 'home', Float64MultiArray, ros_publish_position)
    )
    drone.telemetry.register_imu_handler(
        handle_publisher(node, 'imu', Float32MultiArray, ros_publish_imu)
    )
    drone.telemetry.register_in_air_handler(
        handle_publisher(node, 'in_air', Bool, ros_publish_bool)
    )
    drone.telemetry.register_landed_state_handler(
        handle_publisher(node, 'landed_state', String, ros_publish_landed_state)
    )
    drone.telemetry.register_odometry_handler(
        handle_publisher(node, 'odometry', String, ros_publish_odometry)
    )
    drone.telemetry.register_position_handler(
        handle_publisher(node, 'position', Float64MultiArray, ros_publish_position)
    )
    drone.telemetry.register_position_velocity_ned_handler(
        handle_publisher(node, 'position_velocity_ned', Float32MultiArray, ros_publish_position_velocity_ned)
    )
    drone.telemetry.register_raw_gps_handler(
        handle_publisher(node, 'raw_gps', Float64MultiArray, ros_publish_raw_gps)
    )
    drone.telemetry.register_raw_imu_handler(
        handle_publisher(node, 'raw_imu', Float32MultiArray, ros_publish_imu)
    )
    drone.telemetry.register_rc_status_handler(
        handle_publisher(node, 'rc_status', Float32MultiArray, ros_publish_rc_status)
    )
    drone.telemetry.register_scaled_imu_handler(
        handle_publisher(node, 'scaled_imu', Float32MultiArray, ros_publish_imu)
    )
    drone.telemetry.register_scaled_pressure_handler(
        handle_publisher(node, 'scaled_pressure', Float32MultiArray, ros_publish_scaled_pressure)
    )
    drone.telemetry.register_status_text_handler(
        handle_publisher(node, 'status_text', String, ros_publish_status_text)
    )
    drone.telemetry.register_unix_epoch_time_handler(
        handle_publisher(node, 'unix_epoch_time', UInt64, ros_publish_unix_epoch_time)
    )
    drone.telemetry.register_velocity_ned_handler(
        handle_publisher(node, 'velocity_ned', Float32MultiArray, ros_publish_velocity_ned)
    )
    drone.telemetry.register_vtol_state_handler(
        handle_publisher(node, 'vtol_state', String, ros_publish_vtol_state)
    )


def main(args=None):
    rclpy.init(args=args)
    drone = SimpleDrone("udp://:14540")
    node = Node("drone")
    register_telemetry_ros(node, drone)
    print("registered")
    time.sleep(2)
    drone.arm()
    print("armed")
    time.sleep(2)
    drone.takeoff()
    print("takeoff")
    """while True:
        """
    time.sleep(15)
    # drone.stop()
    print("loop start")
    drone.start_loop()
    print("loop done")
    time.sleep(10)
    print("teardown")
    rclpy.shutdown()
    drone.teardown()
    print("complete")


if __name__ == "__main__":
    main()
    exit()