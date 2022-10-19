from asyncio import AbstractEventLoop
from logging import Logger
from typing import List, Dict, Any, Optional
from functools import partial
import time

from mavsdk import System, telemetry

from .abstract_base_plugin import AbstractBasePlugin


class Telemetry(AbstractBasePlugin):
    """
    Allow users to get vehicle telemetry and state information (e.g. battery, GPS, RC connection, flight mode etc.)
     and set telemetry update rates.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("telemetry", system, loop, logger)

        self._all_methods = self._get_methods()
        self._rate_set_methods = self._get_set_methods(self._all_methods)
        self._getter_methods = self._get_get_methods(self._all_methods)
        self._async_gen_methods = self._remove_set_get_methods(
            self._all_methods, self._rate_set_methods, self._getter_methods
        )

        self._async_gen_data: Dict[str, Any] = {
            key: None for key in self._async_gen_methods
        }
        self._async_rate_data: Dict[str, float] = {
            key: 1.0 for key in self._rate_set_methods
        }
        self._rate_last_times: Dict[str, Optional[float]] = {
            key: None for key in self._rate_set_methods
        }
        self._start_async_gen_telemetry()

        self._getter_data = self._init_getter_data()

        self._end_init()

    def _get_methods(self) -> List[str]:
        return [
            func
            for func in dir(self._system.telemetry)
            if callable(getattr(self._system.telemetry, func))
            and not func.startswith("_")
        ]

    def _get_methods_startswith(self, methods: List[str], starts_with: str) -> List:
        return [func for func in methods if func.startswith(starts_with)]

    def _get_set_methods(self, methods: List) -> List:
        return self._get_methods_startswith(methods, "set")

    def _get_get_methods(self, methods: List) -> List:
        return self._get_methods_startswith(methods, "get")

    def _remove_set_get_methods(
        self, methods: List, rate_methods, getter_methods
    ) -> List:
        for method in rate_methods:
            methods.remove(method)
        for method in getter_methods:
            methods.remove(method)
        return methods

    async def _async_gen_wrapper(self, func: str) -> None:
        async for data in getattr(self._system.telemetry, func)():
            if data != self._async_gen_data[func]:
                self._async_gen_data[func] = data

            if func in self._rate_set_methods:
                prev_time = self._rate_last_times[func]
                current_time = time.time()

                # Need one cycle to calculate
                if prev_time is None:
                    self._rate_last_times[func] = current_time
                    continue

                delta_secs = current_time - prev_time

                # Average the current value and the last value if they are close enough to account for minor variations?
                new_hz = 1 / delta_secs
                current_hz = self._async_rate_data[func]
                if abs(new_hz - current_hz) <= 0.5:
                    new_hz = (new_hz + current_hz) / 2
                self._async_rate_data[func] = new_hz

    def _start_async_gen_telemetry(self) -> None:
        for func in self._async_gen_methods:
            self._submit_generator(partial(self._async_gen_wrapper, func))

    def _init_getter_data(self) -> Dict:
        data: Dict[str, Any] = {}
        for func in self._getter_methods:
            data[func] = None
        return data

    def _get_getter_data(self, func: str, timeout: float) -> Any:
        if self._getter_data[func] is None:
            opt_data = self._submit_blocking_coroutine(
                getattr(self._system.telemetry, func)(), timeout=timeout
            )

            if opt_data is None:
                self._logger.error(
                    f'Failed to get telemetry data for function "{func}" with a timeout of {timeout} seconds'
                )

            self._getter_data[func] = opt_data  # Value is still None if timeout
        return self._getter_data[func]

    # get methods
    # ==========================================================================================

    def get_gps_global_origin(self, timeout: float = 1.0) -> telemetry.GpsGlobalOrigin:
        """
        Get the GPS location of where the estimator has been initialized.
        :return: telemetry.GpsGlobalOrigin ; Gets the Global Origin in latitude, longitude and altitude
        """
        return self._get_getter_data("get_gps_global_origin", timeout)

    # async gen method data
    # ==========================================================================================
    @property
    def actuator_control_target(self) -> telemetry.ActuatorControlTarget:
        """
        Get the next actuator control target
        :return: telemetry.ActuatorControlTarget ; the next control target
        """
        return self._async_gen_data["actuator_control_target"]

    @property
    def actuator_output_start(self) -> telemetry.ActuatorOutputStatus:
        """
        Subscribe to ‘actuator output status’ updates.
        :return: telemetry.ActuatorOutputStatus ; The next actuator output status
        """
        return self._async_gen_data["actuator_output_status"]

    @property
    def armed(self) -> bool:
        """
        Subscribe to armed updates.
        :return: bool ; The next ‘armed’ state
        """
        return self._async_gen_data["armed"]

    @property
    def attitude_angular_velocity_body(self) -> telemetry.AngularVelocityBody:
        """
        Subscribe to ‘attitude’ updates (angular velocity)
        :return: telemetry.AngularVelocityBody ; The next angular velocity (rad/s)
        """
        return self._async_gen_data["attitude_angular_velocity_body"]

    @property
    def attitude_euler(self) -> telemetry.EulerAngle:
        """
        Subscribe to ‘attitude’ updates (Euler).
        :return: telemetry.EulerAngle ; The next attitude (Euler)
        """
        return self._async_gen_data["attitude_euler"]

    @property
    def attitude_quaternion(self) -> telemetry.Quaternion:
        """
        Subscribe to ‘attitude’ updates (quaternion).
        :return: telemetry.Quaternion ;  The next attitude (quaternion)
        """
        return self._async_gen_data["attitude_quaternion"]

    @property
    def battery(self) -> telemetry.Battery:
        """
        Subscribe to ‘battery’ updates.
        :return: telemetry.Battery ; The next ‘battery’ state
        """
        return self._async_gen_data["battery"]

    @property
    def camera_attitude_euler(self) -> telemetry.EulerAngle:
        """
        Subscribe to ‘camera attitude’ updates (Euler).
        :return: telemetry.EulerAngle ; The next camera attitude (Euler)
        """
        return self._async_gen_data["camera_attitude_euler"]

    @property
    def camera_attitude_quaternion(self) -> telemetry.Quaternion:
        """
        Subscribe to ‘camera attitude’ updates (quaternion).
        :return: telemetry.Quaternion ; The next camera attitude (quaternion)
        """
        return self._async_gen_data["camera_attitude_quaternion"]

    @property
    def distance_sensor(self) -> telemetry.DistanceSensor:
        """
        Subscribe to ‘Distance Sensor’ updates.
        :return: telemetry.DistanceSensor ; The next Distance Sensor status
        """
        return self._async_gen_data["distance_sensor"]

    @property
    def fixedwing_metrics(self) -> telemetry.FixedwingMetrics:
        """
        Subscribe to ‘fixedwing metrics’ updates.
        :return: telemetry.FixedwingMetrics ; The next fixedwing metrics
        """
        return self._async_gen_data["fixedwing_metrics"]

    @property
    def flight_mode(self) -> telemetry.FlightMode:
        """
        Subscribe to ‘flight mode’ updates.
        :return: telemetry.FlightMode ; The next flight mode
        """
        return self._async_gen_data["flight_mode"]

    @property
    def gps_info(self) -> telemetry.GpsInfo:
        """
        Subscribe to ‘GPS info’ updates.
        :return: telemetry.GpsInfo ; The next ‘GPS info’ state
        """
        return self._async_gen_data["gps_info"]

    @property
    def ground_truth(self) -> telemetry.GroundTruth:
        """
        Subscribe to ‘ground truth’ updates.
        :return: telemetry.GroundTruth ; Ground truth position information available in simulation
        """
        return self._async_gen_data["ground_truth"]

    @property
    def heading(self) -> telemetry.Heading:
        """
        Subscribe to ‘Heading’ updates.
        :return: telemetry.Heading ; The next heading (yaw) in degrees
        """
        return self._async_gen_data["heading"]

    @property
    def health(self) -> telemetry.Health:
        """
        Subscribe to ‘health’ updates.
        :return: telemetry.Health ; The next ‘health’ state
        """
        return self._async_gen_data["health"]

    @property
    def health_all_ok(self) -> bool:
        """
        Subscribe to ‘HealthAllOk’ updates
        :return: bool ; The next ‘health all ok’ status
        """
        return self._async_gen_data["health_all_ok"]

    @property
    def home(self) -> telemetry.Position:
        """
        Subscribe to ‘home position’ updates.
        :return: telemetry.Position ; The next home position
        """
        return self._async_gen_data["home"]

    @property
    def imu(self) -> telemetry.Imu:
        """
        Subscribe to ‘IMU’ updates (in SI units in NED body frame).
        :return: telemetry.Imu ; The next IMU status
        """
        return self._async_gen_data["imu"]

    @property
    def in_air(self) -> bool:
        """
        Subscribe to in-air updates.
        :return: bool ; The next ‘in-air’ state
        """
        return self._async_gen_data["in_air"]

    @property
    def landed_state(self) -> telemetry.LandedState:
        """
        Subscribe to landed state updates
        :return: telemetry.LandedState ; The next ‘landed’ state
        """
        return self._async_gen_data["landed_state"]

    @property
    def odometry(self) -> telemetry.Odometry:
        """
        Subscribe to ‘odometry’ updates.
        :return: telemetry.Odometry ; The next odometry status
        """
        return self._async_gen_data["odometry"]

    @property
    def position(self) -> telemetry.Position:
        """
        Subscribe to ‘position’ updates.
        :return: telemetry.Position ; The next position
        """
        return self._async_gen_data["position"]

    @property
    def position_velocity_ned(self) -> telemetry.PositionVelocityNed:
        """
        Subscribe to ‘position velocity’ updates.
        :return: telemetry.PositionVelocityNed ; The next position and velocity status
        """
        return self._async_gen_data["position_velocity_ned"]

    @property
    def raw_gps(self) -> telemetry.RawGps:
        """
        Subscribe to ‘Raw GPS’ updates.
        :return: telemetry.RawGps ; The next ‘Raw GPS’ state. Warning: this is an advanced feature, use Position updates
        to get the location of the drone!
        """
        return self._async_gen_data["raw_gps"]

    @property
    def raw_imu(self) -> telemetry.Imu:
        """
        Subscribe to ‘Raw IMU’ updates.
        :return: telemetry.Imu ; The next raw IMU status
        """
        return self._async_gen_data["raw_imu"]

    @property
    def rc_status(self) -> telemetry.RcStatus:
        """
        Subscribe to ‘RC status’ updates.
        :return: telemetry.RcStatus ; The next RC status
        """
        return self._async_gen_data["rc_status"]

    @property
    def scaled_imu(self) -> telemetry.Imu:
        """
        Subscribe to ‘Scaled IMU’ updates.
        :return: telemetry.Imu ; The next scaled IMU status
        """
        return self._async_gen_data["scaled_imu"]

    @property
    def scaled_pressure(self) -> telemetry.ScaledPressure:
        """
        Subscribe to ‘Scaled Pressure’ updates.
        :return: telemetry.ScaledPressure ; The next scaled pressure status
        """
        return self._async_gen_data["scaled_pressure"]

    @property
    def status_text(self) -> telemetry.StatusText:
        """
        Subscribe to ‘status text’ updates.
        :return: telemetry.StatusText ; Status text information type
        """
        return self._async_gen_data["status_text"]

    @property
    def unix_epoch_time(self) -> int:
        """
        Returns the current unix epoch time
        :return: int ; unix epoch time
        """
        return self._async_gen_data["unix_epoch_time"]

    @property
    def velocity_ned(self) -> telemetry.VelocityNed:
        """
        Returns the Velocity in NED coordinate
        :return: telemetry.VelocityNed ; Velocity in NED coordinates
        """
        return self._async_gen_data["velocity_ned"]

    @property
    def vtol_state(self) -> telemetry.VtolState:
        """
        Returns the vtol state
        :return: telemetry.VtolState ; Enumeration of the vtol state
        """
        return self._async_gen_data["vtol_state"]

    # rate setter methods
    # ==========================================================================================
    def set_rate_actuator_target(self, rate: float) -> None:
        """
        Set rate to ‘actuator control target’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(
            self._system.telemetry.set_rate_actuator_control_target(rate)
        )

    def set_rate_actuator_output_status(self, rate: float) -> None:
        """
        Set rate to 'actuator output status' updates
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(
            self._system.telemetry.set_rate_actuator_output_status(rate)
        )

    def set_rate_attitude(self, rate: float) -> None:
        """
        Set rate to 'attitude' updates
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_attitude(rate))

    def set_rate_battery(self, rate: float) -> None:
        """
        Set rate to 'battery' updates
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_battery(rate))

    def set_rate_camera_attitude(self, rate: float) -> None:
        """
        Set rate of camera attitude updates
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_camera_attitude(rate))

    def set_rate_distance_sensor(self, rate: float) -> None:
        """
        Set rate to ‘Distance Sensor’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_distance_sensor(rate))

    def set_rate_fixedwing_metrics(self, rate: float) -> None:
        """
        Set rate to ‘fixedwing metrics’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_fixedwing_metrics(rate))

    def set_rate_gps_info(self, rate: float) -> None:
        """
        Set rate to ‘GPS info’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_gps_info(rate))

    def set_rate_ground_truth(self, rate: float) -> None:
        """
        Set rate to ‘ground truth’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_ground_truth(rate))

    def set_rate_home(self, rate: float) -> None:
        """
        Set rate to ‘home position’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_home(rate))

    def set_rate_imu(self, rate: float) -> None:
        """
        Set rate to ‘IMU’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_imu(rate))

    def set_rate_in_air(self, rate: float) -> None:
        """
        Set rate to in-air updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_in_air(rate))

    def set_rate_landed_state(self, rate: float) -> None:
        """
        Set rate to landed state updates
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_landed_state(rate))

    def set_rate_odometry(self, rate: float) -> None:
        """
        Set rate to ‘odometry’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_odometry(rate))

    def set_rate_position(self, rate: float) -> None:
        """
        Set rate to ‘position’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_position(rate))

    def set_rate_position_velocity_ned(self, rate: float) -> None:
        """
        Set rate to ‘position velocity’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(
            self._system.telemetry.set_rate_position_velocity_ned(rate)
        )

    def set_rate_raw_imu(self, rate: float) -> None:
        """
        Set rate to ‘Raw IMU’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_raw_imu(rate))

    def set_rate_rc_status(self, rate: float) -> None:
        """
        Set rate to ‘RC status’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_rc_status(rate))

    def set_rate_scaled_imu(self, rate: float) -> None:
        """
        Set rate to ‘Scaled IMU’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_scaled_imu(rate))

    def set_rate_unix_epoch_time(self, rate: float) -> None:
        """
        Set rate to ‘unix epoch time’ updates.
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_unix_epoch_time(rate))

    def set_rate_velocity_ned(self, rate: float) -> None:
        """
        Set rate to ‘ground speed’ updates (NED).
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_velocity_ned(rate))

    def set_rate_vtol_state(self, rate: float) -> None:
        """
        Set rate to VTOL state updates
        :param rate: double ; The requested rate in Hertz
        :return: None
        """
        self._submit_coroutine(self._system.telemetry.set_rate_vtol_state(rate))

    # rate getter methods
    # ==========================================================================================

    def get_rate_actuator_target(self) -> float:
        """
        Get 'actuator control target' updates rate in Hertz

        :return: 'actuator control target' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_actuator_target"]

    def get_rate_actuator_output_status(self) -> float:
        """
        Get 'actuator output status' updates rate in Hertz

        :return: 'actuator output status' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_actuator_output_status"]

    def get_rate_attitude(self) -> float:
        """
        Get 'attitude' updates rate in Hertz

        :return: 'attitude' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_attitude"]

    def get_rate_battery(self) -> float:
        """
        Get 'battery' updates rate in Hertz

        :return: 'battery' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_battery"]

    def get_rate_camera_attitude(self) -> float:
        """
        Get 'camera attitude' updates rate in Hertz

        :return: 'camera attitude' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_camera_attitude"]

    def get_rate_distance_sensor(self) -> float:
        """
        Get 'distance sensor' updates rate in Hertz

        :return: 'distance sensor' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_distance_sensor"]

    def get_rate_fixedwing_metrics(self) -> float:
        """
        Get 'fixedwing metrics' updates rate in Hertz

        :return: 'fixedwing metrics' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_fixedwing_metrics"]

    def get_rate_gps_info(self) -> float:
        """
        Get 'GPS info' updates rate in Hertz

        :return: 'GPS info' updates rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_gps_info"]

    def get_rate_ground_truth(self) -> float:
        """
        Get 'ground truth' update rate in Hertz

        :return: 'ground truth' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_ground_truth"]

    def get_rate_home(self) -> float:
        """
        Get 'home position' update rate in Hertz

        :return: 'home position' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_home"]

    def get_rate_imu(self) -> float:
        """
        Get 'IMU' update rate in Hertz

        :return: 'IMU' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_imu"]

    def get_rate_in_air(self) -> float:
        """
        Get 'in-air' update rate in Hertz

        :return: 'in-air' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_in_air"]

    def get_rate_landed_state(self) -> float:
        """
        Get 'landed-state' update rate in Hertz

        :return: 'landed-state' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_landed_state"]

    def get_rate_odometry(self) -> float:
        """
        Get 'odometry' update rate in Hertz

        :return: 'odometry' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_odometry"]

    def get_rate_position(self) -> float:
        """
        Get 'position' update rate in Hertz

        :return: 'position' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_position"]

    def get_rate_position_velocity_ned(self) -> float:
        """
        Get 'position velocity' update rate in Hertz

        :return: 'position velocity' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_position_velocity_ned"]

    def get_rate_raw_imu(self) -> float:
        """
        Get 'raw IMU' update rate in Hertz

        :return: 'Raw IMU' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_raw_imu"]

    def get_rate_rc_status(self) -> float:
        """
        Get 'RC status' update rate in Hertz

        :return: 'RC status' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_rc_status"]

    def get_rate_scaled_imu(self) -> float:
        """
        Get 'scaled IMU' update rate in Hertz

        :return: 'scaled IMU' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_scaled_imu"]

    def get_rate_unix_epoch_time(self) -> float:
        """
        Get 'unix epoch time' update rate in Hertz

        :return: 'unix epoch time' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_unix_epoch_time"]

    def get_rate_velocity_ned(self) -> float:
        """
        Get 'ground speed' update rate in Hertz (NED)

        :return: 'ground speed' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_velocity_ned"]

    def get_rate_vtol_state(self) -> float:
        """
        Get 'VTOL state' update rate in Hertz

        :return: 'VTOL state' update rate in Hertz
        :rtype: float
        """
        return self._async_rate_data["set_rate_vtol_state"]
