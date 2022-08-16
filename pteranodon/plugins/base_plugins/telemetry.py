import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from time import sleep
from typing import List, Dict, Any

from mavsdk import System, telemetry

from .abstract_base_plugin import AbstractBasePlugin


class Telemetry(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("telemetry", system, loop, logger)

        self._all_methods = self._get_methods()
        self._rate_set_methods = self._get_set_methods(self._all_methods)
        self._getter_methods = self._get_get_methods(self._all_methods)
        self._async_gen_methods = self._remove_set_get_methods(self._all_methods, self._rate_set_methods, self._getter_methods)

        self._async_gen_data = self._make_async_gen_data()
        self._async_gen_tasks = self._start_async_gen_telemetry()

        self._getter_data = self._init_getter_data()

    def _get_methods(self) -> List:
        return [func for func in dir(self._system.telemetry) if callable(getattr(self._system.telemetry, func)) and not func.startswith("_")]

    def _get_methods_startswith(self, methods: List, starts_with: str) -> List:
        return [func for func in methods if func.startswith(starts_with)]

    def _get_set_methods(self, methods: List) -> List:
        return self._get_methods_startswith(methods, "set")
    
    def _get_get_methods(self, methods: List) -> List:
        return self._get_methods_startswith(methods, "get")

    def _remove_set_get_methods(self, methods: List, rate_methods, getter_methods) -> List:
        for method in rate_methods:
            methods.remove(method)
        for method in getter_methods:
            methods.remove(method)
        return methods

    def _make_async_gen_data(self) -> Dict:
        data = {}
        for func in self._async_gen_methods:
            data[func] = None
        return data

    async def _async_gen_wrapper(self, func: str) -> None:
        async for data in getattr(self._system.telemetry, func)():
            if data != self._async_gen_data[func]:
                self._async_gen_data[func] = data

    def _start_async_gen_telemetry(self) -> Dict:
        tasks = {}
        for func in self._async_gen_methods:
            tasks[func] = asyncio.ensure_future(self._async_gen_wrapper(func), loop=self._loop)
        return tasks

    def _init_getter_data(self) -> Dict:
        data = {}
        for func in self._getter_methods:
            data[func] = None
        return data

    def _get_getter_data(self, func: str) -> Any:
        if self._getter_data[func] is None:
            task = asyncio.ensure_future(getattr(self._system.telemetry, func)(), loop=self._loop)
            while not task.done():
                sleep(0.000001)
            self._getter_data = task.result()
        return self._getter_data[func]

    # get methods
    # ==========================================================================================
    @property
    def get_gps_global_origin(self) -> telemetry.GpsGlobalOrigin:
        return self._get_getter_data("get_gps_global_origin")

    # async gen method data
    # ==========================================================================================
    @property
    def actuator_control_target(self) -> telemetry.ActuatorControlTarget:
        return self._async_gen_data["actuator_control_target"]

    @property
    def actuator_output_start(self) -> telemetry.ActuatorOutputStatus:
        return self._async_gen_data["actuator_output_status"]

    @property
    def armed(self) -> bool:
        return self._async_gen_data["armed"]

    @property
    def attitude_angular_velocity_body(self) -> telemetry.AngularVelocityBody:
        return self._async_gen_data["attitude_angular_velocity_body"]

    @property
    def attitude_euler(self) -> telemetry.EulerAngle:
        return self._async_gen_data["attitude_euler"]

    @property
    def attitude_quaternion(self) -> telemetry.Quaternion:
        return self._async_gen_data["attitude_quaternion"]

    @property
    def battery(self) -> telemetry.Battery:
        return self._async_gen_data["battery"]

    @property
    def camera_attitude_euler(self) -> telemetry.EulerAngle:
        return self._async_gen_data["camera_attitude_euler"]

    @property
    def camera_attitude_quaternion(self) -> telemetry.Quaternion:
        return self._async_gen_data["camera_attitude_quaternion"]

    @property
    def distance_sensor(self) -> telemetry.DistanceSensor:
        return self._async_gen_data["distance_sensor"]

    @property
    def fixedwing_metrics(self) -> telemetry.FixedwingMetrics:
        return self._async_gen_data["fixedwing_metrics"]

    @property
    def flight_mode(self) -> telemetry.FlightMode:
        return self._async_gen_data["flight_mode"]

    @property
    def gps_info(self) -> telemetry.GpsInfo:
        return self._async_gen_data["gps_info"]
    
    @property
    def ground_truth(self) -> telemetry.GroundTruth:
        return self._async_gen_data["ground_truth"]

    @property
    def heading(self) -> telemetry.Heading:
        return self._async_gen_data["heading"]

    @property
    def health(self) -> telemetry.Health:
        return self._async_gen_data["health"]

    @property
    def health_all_ok(self) -> bool:
        return self._async_gen_data["health_all_ok"]

    @property
    def home(self) -> telemetry.Position:
        return self._async_gen_data["home"]

    @property
    def imu(self) -> telemetry.Imu:
        return self._async_gen_data["imu"]

    @property
    def in_air(self) -> bool:
        return self._async_gen_data["in_air"]

    @property
    def landed_state(self) -> telemetry.LandedState:
        return self._async_gen_data["landed_state"]

    @property
    def odometry(self) -> telemetry.Odometry:
        return self._async_gen_data["odometry"]

    @property
    def position(self) -> telemetry.Position:
        return self._async_gen_data["position"]

    @property
    def position_velocity_ned(self) -> telemetry.PositionVelocityNed:
        return self._async_gen_data["position_velocity_ned"]

    @property
    def raw_gps(self) -> telemetry.RawGps:
        return self._async_gen_data["raw_gps"]

    @property
    def raw_imu(self) -> telemetry.Imu:
        return self._async_gen_data["raw_imu"]

    @property
    def rc_status(self) -> telemetry.RcStatus:
        return self._async_gen_data["rc_status"]

    @property
    def scaled_imu(self) -> telemetry.Imu:
        return self._async_gen_data["scaled_imu"]

    @property
    def scaled_pressure(self) -> telemetry.ScaledPressure:
        return self._async_gen_data["scaled_pressure"]

    @property
    def status_text(self) -> telemetry.StatusText:
        return self._async_gen_data["status_text"]
    
    @property
    def unix_epoch_time(self) -> int:
        return self._async_gen_data["unix_epoch_time"]

    @property
    def velocity_ned(self) -> telemetry.VelocityNed:
        return self._async_gen_data["velocity_ned"]

    @property
    def vtol_state(self) -> telemetry.VtolState:
        return self._async_gen_data["vtol_state"]

    # rate setter methods
    # ==========================================================================================
    def set_rate_actuator_target(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_actuator_control_target(rate)))

    def set_rate_actuator_output_status(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_actuator_output_status(rate)))
    
    def set_rate_attitude(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_attitude(rate)))
    
    def set_rate_battery(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_battery(rate)))
    
    def set_rate_camera_attitude(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_camera_attitude(rate)))
    
    def set_rate_distance_sensor(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_distance_sensor(rate)))
    
    def set_rate_fixedwing_metrics(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_fixedwing_metrics(rate)))
    
    def set_rate_gps_info(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_gps_info(rate)))
    
    def set_rate_ground_truth(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_ground_truth(rate)))
    
    def set_rate_home(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_home(rate)))
    
    def set_rate_imu(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_imu(rate)))

    def set_rate_in_air(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_in_air(rate)))
    
    def set_rate_landed_state(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_landed_state(rate)))
    
    def set_rate_odometry(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_odometry(rate)))
    
    def set_rate_position(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_position(rate)))
    
    def set_rate_position_velocity_ned(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_position_velocity_ned(rate)))

    def set_rate_raw_imu(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_raw_imu(rate)))
    
    def set_rate_rc_status(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_rc_status(rate)))
    
    def set_rate_scaled_imu(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_scaled_imu(rate)))
    
    def set_rate_unix_epoch_time(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_unix_epoch_time(rate)))
    
    def set_rate_velocity_ned(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_velocity_ned(rate)))

    def set_rate_vtol_state(self, rate: float) -> None:
        super().submit_task(asyncio.ensure_future(self._system.telemetry.set_rate_vtol_state(rate)))
