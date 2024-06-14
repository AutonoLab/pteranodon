from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import (
    String,
    Bool,
    Float32MultiArray,
    Float64MultiArray,
    Float64,
    UInt8MultiArray,
    MultiArrayDimension,
    UInt64,
)
from mavsdk.telemetry import (
    AngularVelocityBody,
    EulerAngle,
    Quaternion,
    Battery,
    DistanceSensor,
    FixedwingMetrics,
    FlightMode,
    GpsInfo,
    GroundTruth,
    Heading,
    Health,
    Position,
    Imu,
    Odometry,
    LandedState,
    PositionVelocityNed,
    RawGps,
    RcStatus,
    ScaledPressure,
    StatusText,
    VelocityNed,
    VtolState,
    ActuatorControlTarget,
)
from pteranodon.plugins.base_plugins.telemetry import Telemetry
from ..prefix import PREFIX
from .handle_publisher import handle_publisher


# bool
def _ros_publish_bool(publisher: Publisher, data: bool):
    """Takes input of Publisher, and bool, publishes Bool to ros topic"""
    msg = Bool()
    msg.data = data
    publisher.publish(msg)


# ActuatorControlTarget
def _ros_publish_actuator_control_target(
    publisher: Publisher, data: ActuatorControlTarget
) -> None:
    """No test data
    Takes input of Publisher, and ActuatorControlTarget, consisting of:
        int32 group,
        list[float] controls
    publishes Float32MultiArray[group, controls] to ros topic
    """
    print(data)
    msg = Float32MultiArray()
    msg.data = [data.group] + list(data.controls)
    publisher.publish(msg)


# AngularVelocityBody
def _ros_publish_velocity_body(publisher: Publisher, data: AngularVelocityBody) -> None:
    """
    Takes input of Publisher, and AngularVelocityBody, consisting of:
        float roll_rad_s,
        float pitch_rad_s,
        float yaw_rad_s
    publishes Float32MultiArray[roll, pitch, yaw] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [data.roll_rad_s, data.pitch_rad_s, data.yaw_rad_s]
    publisher.publish(msg)


# EulerAngle
def _ros_publish_euler(publisher: Publisher, data: EulerAngle) -> None:
    """
    Takes input of Publisher, and EulerAngle, consisting of:
        float roll_deg,
        float pitch_deg,
        float yaw_rdeg
    publishes Float32MultiArray[roll, pitch, yaw] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [data.roll_deg, data.pitch_deg, data.yaw_deg]
    publisher.publish(msg)


# Quaternion
def _ros_publish_quaternion(publisher: Publisher, data: Quaternion) -> None:
    """
    Takes input of Publisher, and AngularVelocityBody, consisting of:
        float w,
        float x,
        float y,
        float z,
        uint64 timestamp_us
    publishes Float32MultiArray[w, x, y, z, float(timestamp_us)] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [data.w, data.x, data.y, data.z, float(data.timestamp_us)]
    publisher.publish(msg)


# Battery
def _ros_publish_battery(publisher: Publisher, data: Battery) -> None:
    """
    Takes input of Publisher, and Battery, consisting of:
        float voltage_v,
        float remaining_percent,
    publishes Float32MultiArray[voltage, remaining_percent] to ros topic
    """
    msg = Float32MultiArray()
    # could add float(data.id) to msg data if necessary
    msg.data = [data.voltage_v, data.remaining_percent]
    publisher.publish(msg)


# DistanceSensor
def _ros_publish_distance(publisher: Publisher, data: DistanceSensor) -> None:
    """
    Takes input of Publisher, and DistanceSensor, consisting of:
        float minimum_distance_m,
        float maximum_distance_m,
        float current_distance_m
    publishes Float32MultiArray[minimum, maximum, current] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [
        data.minimum_distance_m,
        data.maximum_distance_m,
        data.current_distance_m,
    ]
    publisher.publish(msg)


# FixedwingMetrics <-- probably not necessary for drone use
def _ros_publish_fixedwing(publisher: Publisher, data: FixedwingMetrics) -> None:
    """
    Takes input of Publisher, and AngularVelocityBody, consisting of:
        float airspeed_m_s,
        float throttle_percentage,
        float climb_rate_m_s
    publishes Float32MultiArray[airspeed, throttle, climb_rate] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [data.airspeed_m_s, data.throttle_percentage, data.climb_rate_m_s]
    publisher.publish(msg)


# FlightMode (ENUM)
def _ros_publish_flight_mode(publisher: Publisher, data: FlightMode) -> None:
    """
    Takes input of Publisher, and FlightMode, enum:
        UNKNOWN
        READY
        TAKEOFF
        HOLD
        MISSION
        RETURN_TO_LAUNCH
        LAND
        OFFBOARD
        FOLLOW_ME
        MANUAL
        ALTCTL
        POSCTL
        ACRO
        STABILIZED
        RATTITUDE
    publishes Float32MultiArray[str(FlightMode)] to ros topic
    """
    msg = String()
    msg.data = data
    publisher.publish(msg)


# GPSInfo
def _ros_publish_gps_info(publisher: Publisher, data: GpsInfo) -> None:
    """
    Takes input of Publisher, and GpsInfo, consisting of:
        int32 num_satellites
        FixType fix_type enum:
            NO_GPS = 0
            NO_FIX = 1
            FIX_2D = 2
            FIX_3D = 3
            FIX_DGPS = 4
            RTK_FLOAT = 5
            RTK_FIXED = 6
    publishes Float32MultiArray[num_satellites, fix_type] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [data.num_satellites, data.fix_type]
    publisher.publish(msg)


# GroundTruth
def _ros_publish_ground_truth(publisher: Publisher, data: GroundTruth) -> None:
    """
    Takes input of Publisher, and GroundTruth, consisting of:
        double latitude_deg,
        double longitude_deg,
        float absolute_altitude_m
    publishes Float64MultiArray[latitude, longitude, absolute_altitude] to ros topic
    """
    msg = Float64MultiArray()
    msg.data = [data.latitude_deg, data.longitude_deg, data.absolute_altitude_m]
    publisher.publish(msg)


# Heading
def _ros_publish_heading(publisher: Publisher, data: Heading) -> None:
    """
    Takes input of Publisher, and AngularVelocityBody, consisting of:
        double heading_deg
    publishes Float64(heading_deg) to ros topic
    """
    msg = Float64()
    msg.data = data.heading_deg
    publisher.publish(msg)


# Health
def _ros_publish_health(publisher: Publisher, data: Health) -> None:
    """
    Takes input of Publisher, and Health, consisting of:
        bool is_armable,
        bool is_gyrometer_calibration_ok,
        bool is_accelerometer_calibration_ok,
        bool is_magnetometer_calibration_ok,
        bool is_local_position_ok,
        bool is_global_position_ok,
        bool is_home_position_ok
    publishes UInt8MultiArray[armable, gyrometer, accelerometer, magnetometer, local_position, global_position, home_position] to ros topic
    """
    msg = UInt8MultiArray()
    msg.data = [
        data.is_armable,
        data.is_gyrometer_calibration_ok,
        data.is_accelerometer_calibration_ok,
        data.is_magnetometer_calibration_ok,
        data.is_local_position_ok,
        data.is_global_position_ok,
        data.is_home_position_ok,
    ]
    publisher.publish(msg)


# Position
def _ros_publish_position(publisher: Publisher, data: Position) -> None:
    """
    Takes input of Publisher, and Position, consisting of:
        double latitude_deg,
        double longitude_deg,
        float absolute_altitude_m,
        float relative_altitude_m
    publishes Float64MultiArray[latitude, longitude, absolute_altitude, relative_altitude] to ros topic
    """
    msg = Float64MultiArray()
    msg.data = [
        data.latitude_deg,
        data.longitude_deg,
        data.absolute_altitude_m,
        data.relative_altitude_m,
    ]
    publisher.publish(msg)


# Imu
def _ros_publish_imu(publisher: Publisher, data: Imu) -> None:
    """
    Takes input of Publisher, and Imu, consisting of:
        AccelerationFrd: (float forward_m_s2, float right_m_s2, float down_m_s2),
        AngularVelocityFrd: (float forward_rad_s, float right_rad_s, float down_rad_s),
        MagneticFieldFrd: (float forward_gauss, float right_gauss, float down_gauss),
        float temperature_degc,
        uint64 timestamp_us
    publishes Float32MultiArray[forward_m_s2, right_m_s2, down_m_s2,
                                forward_rad_s, right_rad_s, down_rad_s,
                                forward_gauss, right_gauss, down_gauss,
                                temperature_degc, float(timestamp_us)]
                                to ros topic
    """
    msg = Float32MultiArray()
    msg.layout.dim = [
        MultiArrayDimension(label="rows", size=4),
        MultiArrayDimension(label="cols", size=3),
    ]
    msg.data = [
        data.acceleration_frd.forward_m_s2,
        data.acceleration_frd.right_m_s2,
        data.acceleration_frd.down_m_s2,
        data.angular_velocity_frd.forward_rad_s,
        data.angular_velocity_frd.right_rad_s,
        data.angular_velocity_frd.down_rad_s,
        data.magnetic_field_frd.forward_gauss,
        data.magnetic_field_frd.right_gauss,
        data.magnetic_field_frd.down_gauss,
        data.temperature_degc,
        float(data.timestamp_us),
    ]
    publisher.publish(msg)


# LandedState (ENUM)
def _ros_publish_landed_state(publisher: Publisher, data: LandedState) -> None:
    """
    Takes input of Publisher, and LandedState, enum:
        UNKNOWN
        ON_GROUND
        IN_AIR
        TAKING_OFF
        LANDING
    publishes String(LandedState) to ros topic
    """
    msg = String()
    msg.data = str(data)
    publisher.publish(msg)


# MavFrame map
mavframe_map = {"UNDEF": 0.0, "BODY_NED": 1.0, "VISION_NED": 2.0, "ESTIM_NED": 3.0}


# Odometry
def _ros_publish_odometry(publisher: Publisher, data: Odometry) -> None:
    """
    Takes input of Publisher, and Odometry, consisting of:
        uint64 time_usec
        MavFrame frame_id enum:
            UNDEF = 0
            BODY_NED = 1
            VISION_NED = 2
            ESTIM_NED = 3
        MavFrame child_frame_id,
        PositionBody position_body: float x_m, float y_m, float z_m,
        Quaternion q: float w, float x, float y, float z, uint64 timestamp_us,
        VelocityBody velocity_body: float x_m_s, float y_m_s, float z_m_s,
        AngularVelocityBody angular_velocity_body: float roll_rad_s, float pitch_rad_s, yaw_rad_s,
        Covariance pose_covariance: list[float] covariance_matrix,
        Covariance velocity_covariance
    publishes Float32MultiArray[time_usec,
                frame_id,
                child_frame_id,
                position_body.x, position_body.y, position_body.z,
                q.w, q.x, q.y, q.z, float(q.uint64),
                velocity_body.x, velocity_body.y, velocity.z,
                angular_velocity_body.roll, angular_velocity_body.pitch, angular_velocity_body.yaw] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = (
        [
            float(data.time_usec),
            mavframe_map[str(data.frame_id)],
            mavframe_map[str(data.child_frame_id)],
            data.position_body.x_m,
            data.position_body.y_m,
            data.position_body.z_m,
            data.q.w,
            data.q.x,
            data.q.y,
            data.q.z,
            float(data.q.timestamp_us),
            data.velocity_body.x_m_s,
            data.velocity_body.y_m_s,
            data.velocity_body.z_m_s,
            data.angular_velocity_body.roll_rad_s,
            data.angular_velocity_body.pitch_rad_s,
            data.angular_velocity_body.yaw_rad_s,
        ]
        + list(data.pose_covariance.covariance_matrix)
        + list(data.velocity_covariance.covariance_matrix)
    )
    publisher.publish(msg)


# PositionVelocityNED
def _ros_publish_position_velocity_ned(
    publisher: Publisher, data: PositionVelocityNed
) -> None:
    """
    Takes input of Publisher, and PositionVelocityNed, consisting of:
        PositionNed position: (float north_m, float east_m, float down_m),
        VelocityNed velocity: (float north_m_s, float east_m_s, float down_m_s)
    publishes Float32MultiArray[north_m, east_m, down_m, north_m_s, east_m_s, down_m_s] to ros topic
    """
    msg = Float32MultiArray()
    msg.layout.dim = [
        MultiArrayDimension(label="rows", size=2),
        MultiArrayDimension(label="cols", size=3),
    ]
    msg.data = [
        data.position.north_m,
        data.position.east_m,
        data.position.down_m,
        data.velocity.north_m_s,
        data.velocity.east_m_s,
        data.velocity.down_m_s,
    ]
    publisher.publish(msg)


# RawGPS
def _ros_publish_raw_gps(publisher: Publisher, data: RawGps) -> None:
    """
    Takes input of Publisher, and Position, consisting of:
        uint64 timestamp_us,
        double latitude_deg,
        double longitude_deg,
        float absolute_altitude_m,
        float hdop,
        float vdop,
        float velocity_m_s,
        float cog_deg,
        float altitude_ellipsoid_m,
        float horizontal_uncertainty_m,
        float vertical_uncertainty_m,
        float velocity_uncertainty_m_s,
        float heading_uncertainty_deg
    publishes Float64MultiArray[
        float(timestamp_us), latitude, longitude, absolute_altitude,
        hdop, vdop, velocity, cog, altitude_ellipsoid,
        horizontal_uncert, vertical_uncert, velocity_uncert, heading_uncert]
    to ros topic
    """
    msg = Float64MultiArray()
    msg.data = [
        float(data.timestamp_us),
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
        data.yaw_deg,
    ]
    publisher.publish(msg)


# RCStatus
def _ros_publish_rc_status(publisher: Publisher, data: RcStatus) -> None:
    """
    Takes input of Publisher, and RcStatus, consisting of:
        bool was_available_once,
        bool is_available,
        float signal_strength_percentage
    publishes Float32MultiArray[float(was_available), float(is_available), signal_strength_percent] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [
        float(data.was_available_once),
        float(data.is_available),
        data.signal_strength_percent,
    ]
    publisher.publish(msg)


# ScaledPressure
def _ros_publish_scaled_pressure(publisher: Publisher, data: ScaledPressure) -> None:
    """
    Takes input of Publisher, and ScaledPressure, consisting of:
        uint64 timestamp_us,
        float absolute_pressure_hpa,
        float differential_pressure_hpa,
        float temperature_deg,
        float differential_pressure_temperature_deg
    publishes Float32MultiArray[
        float(timestamp_us), absolute_pressure, differential_pressure,
        temperature, differential_pressure_temperature]
    to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [
        float(data.timestamp_us),
        data.absolute_pressure_hpa,
        data.differential_pressure_hpa,
        data.temperature_deg,
        data.differential_pressure_temperature_deg,
    ]
    publisher.publish(msg)


# StatusText
def _ros_publish_status_text(publisher: Publisher, data: StatusText) -> None:
    """
    Takes input of Publisher, and StatusText, consisting of:
        StatusTextType type enum:
            DEBUG
            INFO
            NOTICE
            WARNING
            ERROR
            CRITICAL
            ALERT
            EMERGENCY
        str text,
    publishes String(str(type) + ": " + text) to ros topic
    """
    msg = String()
    msg.data = str(str(data.type) + ": " + str(data.text))
    publisher.publish(msg)


# UnixEpochTime
def _ros_publish_unix_epoch_time(publisher: Publisher, data: int) -> None:
    """Takes input of Publisher, and int, publishes UInt64 to ros topic"""
    msg = UInt64()
    msg.data = data
    publisher.publish(msg)


# VelocityNed
def _ros_publish_velocity_ned(publisher: Publisher, data: VelocityNed) -> None:
    """
    Takes input of Publisher, and VelocityNed, consisting of:
        float north_m_s,
        float east_m_s,
        float down_m_s
    publishes Float32MultiArray[north, east, down] to ros topic
    """
    msg = Float32MultiArray()
    msg.data = [data.north_m_s, data.east_m_s, data.down_m_s]
    publisher.publish(msg)


# VtolState (ENUM) <-- also probably not necessary for our drone
def _ros_publish_vtol_state(publisher: Publisher, data: VtolState) -> None:
    """
    Takes input of Publisher, and VtolState, enum:
        UNDEFINED
        TRANSITION_TO_FW
        TRANSITION_TO_MC
        MC
        FW
    publishes String(str(VtolState)) to ros topic
    """
    msg = String()
    msg.data = str(data)
    publisher.publish(msg)


def register_telemetry_publishers(node: Node, telemetry: Telemetry) -> None:
    """Register handlers for telemetry metrics"""
    telemetry.register_armed_handler(
        handle_publisher(node, PREFIX + "armed", Bool, _ros_publish_bool)
    )
    telemetry.register_actuator_control_target_handler(
        handle_publisher(
            node,
            PREFIX + "actuator_control_target",
            String,
            _ros_publish_actuator_control_target,
        )
    )
    telemetry.register_attitude_angular_velocity_body_handler(
        handle_publisher(
            node,
            PREFIX + "attitude_angular_velocity_body",
            Float32MultiArray,
            _ros_publish_velocity_body,
        )
    )
    telemetry.register_attitude_euler_handler(
        handle_publisher(
            node, PREFIX + "attitude_euler", Float32MultiArray, _ros_publish_euler
        )
    )
    telemetry.register_attitude_quaternion_handler(
        handle_publisher(
            node,
            PREFIX + "attitude_quaternion",
            Float32MultiArray,
            _ros_publish_quaternion,
        )
    )
    telemetry.register_battery_handler(
        handle_publisher(
            node, PREFIX + "battery", Float32MultiArray, _ros_publish_battery
        )
    )
    telemetry.register_camera_attitude_euler_handler(
        handle_publisher(
            node,
            PREFIX + "camera_attitude_euler",
            Float32MultiArray,
            _ros_publish_euler,
        )
    )
    telemetry.register_camera_attitude_quaternion_handler(
        handle_publisher(
            node,
            PREFIX + "camera_attitude_quaternion",
            Float32MultiArray,
            _ros_publish_quaternion,
        )
    )
    telemetry.register_distance_sensor_handler(
        handle_publisher(
            node, PREFIX + "distance_sensor", Float32MultiArray, _ros_publish_distance
        )
    )
    telemetry.register_fixedwing_metrics_handler(
        handle_publisher(
            node,
            PREFIX + "fixedwing_metrics",
            Float32MultiArray,
            _ros_publish_fixedwing,
        )
    )
    telemetry.register_flight_mode_handler(
        handle_publisher(node, PREFIX + "flight_mode", String, _ros_publish_flight_mode)
    )
    telemetry.register_gps_info_handler(
        handle_publisher(
            node, PREFIX + "gps_info", Float32MultiArray, _ros_publish_gps_info
        )
    )
    telemetry.register_ground_truth_handler(
        handle_publisher(
            node, PREFIX + "ground_truth", Float64MultiArray, _ros_publish_ground_truth
        )
    )
    telemetry.register_heading_handler(
        handle_publisher(node, PREFIX + "heading", Float64, _ros_publish_heading)
    )
    telemetry.register_health_all_ok_handler(
        handle_publisher(node, PREFIX + "health_all_ok", Bool, _ros_publish_bool)
    )
    telemetry.register_health_handler(
        handle_publisher(node, PREFIX + "health", UInt8MultiArray, _ros_publish_health)
    )
    telemetry.register_home_handler(
        handle_publisher(
            node, PREFIX + "home", Float64MultiArray, _ros_publish_position
        )
    )
    telemetry.register_imu_handler(
        handle_publisher(node, PREFIX + "imu", Float32MultiArray, _ros_publish_imu)
    )
    telemetry.register_in_air_handler(
        handle_publisher(node, PREFIX + "in_air", Bool, _ros_publish_bool)
    )
    telemetry.register_landed_state_handler(
        handle_publisher(
            node, PREFIX + "landed_state", String, _ros_publish_landed_state
        )
    )
    telemetry.register_odometry_handler(
        handle_publisher(
            node, PREFIX + "odometry", Float32MultiArray, _ros_publish_odometry
        )
    )
    telemetry.register_position_handler(
        handle_publisher(
            node, PREFIX + "position", Float64MultiArray, _ros_publish_position
        )
    )
    telemetry.register_position_velocity_ned_handler(
        handle_publisher(
            node,
            PREFIX + "position_velocity_ned",
            Float32MultiArray,
            _ros_publish_position_velocity_ned,
        )
    )
    telemetry.register_raw_gps_handler(
        handle_publisher(
            node, PREFIX + "raw_gps", Float64MultiArray, _ros_publish_raw_gps
        )
    )
    telemetry.register_raw_imu_handler(
        handle_publisher(node, PREFIX + "raw_imu", Float32MultiArray, _ros_publish_imu)
    )
    telemetry.register_rc_status_handler(
        handle_publisher(
            node, PREFIX + "rc_status", Float32MultiArray, _ros_publish_rc_status
        )
    )
    telemetry.register_scaled_imu_handler(
        handle_publisher(
            node, PREFIX + "scaled_imu", Float32MultiArray, _ros_publish_imu
        )
    )
    telemetry.register_scaled_pressure_handler(
        handle_publisher(
            node,
            PREFIX + "scaled_pressure",
            Float32MultiArray,
            _ros_publish_scaled_pressure,
        )
    )
    telemetry.register_status_text_handler(
        handle_publisher(node, PREFIX + "status_text", String, _ros_publish_status_text)
    )
    telemetry.register_unix_epoch_time_handler(
        handle_publisher(
            node, PREFIX + "unix_epoch_time", UInt64, _ros_publish_unix_epoch_time
        )
    )
    telemetry.register_velocity_ned_handler(
        handle_publisher(
            node, PREFIX + "velocity_ned", Float32MultiArray, _ros_publish_velocity_ned
        )
    )
    telemetry.register_vtol_state_handler(
        handle_publisher(node, PREFIX + "vtol_state", String, _ros_publish_vtol_state)
    )
