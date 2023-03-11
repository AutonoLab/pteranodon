from asyncio import AbstractEventLoop
from logging import Logger
from typing import Callable

from mavsdk import System
from mavsdk.gimbal import ControlMode, ControlStatus, GimbalMode

from .abstract_base_plugin import AbstractBasePlugin


class Gimbal(AbstractBasePlugin):
    """
    Provide control over a gimbal.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("gimbal", system, loop, logger)

        self._submit_generator(self._system.gimbal.control, quit_on_error=True)

        self._end_init()

    def control(self) -> ControlStatus:
        """
        The current control status.

        :return: the current control status
        """
        return self._async_gen_data[self._system.gimbal.control]

    def release_control(self) -> None:
        """
        Release control, such that other components can control the gimbal.
        """

        self._submit_coroutine(self._system.gimbal.release_control())

    def set_mode(self, gimbal_mode: GimbalMode) -> None:
        """
        Sets the desired yaw mode of a gimbal.

        :param gimbal_mode: The mode to be set
        :type gimbal_mode: GimbalMode
        """

        self._submit_coroutine(self._system.gimbal.set_mode(gimbal_mode))

    def set_pitch_and_yaw(self, pitch_deg: float, yaw_deg: float) -> None:
        """
        Set gimbal pitch and yaw angles.

        :param pitch_deg: Pitch angle in degrees (negative points down)
        :type pitch_deg: float

        :param yaw_deg: Yaw angle in degrees (positive is clock-wise, range: -180 to 180 or 0 to 360)
        :type yaw_deg: float
        """

        self._submit_coroutine(
            self._system.gimbal.set_pitch_and_yaw(pitch_deg, yaw_deg)
        )

    def set_pitch_rate_and_yaw_rate(
        self, pitch_rate_deg_s: float, yaw_rate_deg_s: float
    ) -> None:
        """
        Set gimbal angular rates around pitch and yaw axes.

        :param pitch_rate_deg_s: Angular rate around pitch axis in degrees/second (negative downward)
        :type pitch_rate_deg_s: float

        :param yaw_rate_deg_s: Angular rate around yaw axis in degrees/second (positive is clock-wise)
        :type yaw_rate_deg_s: float
        """

        self._submit_coroutine(
            self._system.gimbal.set_pitch_rate_and_yaw_rate(
                pitch_rate_deg_s, yaw_rate_deg_s
            )
        )

    def set_roi_location(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> None:
        """
        Set gimbal region of interest (ROI).

        :param latitude_deg: Latitude in degrees
        :type latitude_deg: float

        :param longitude_deg: Longitude in degrees
        :type longitude_deg: float

        :param altitude_m: Altitude in metres (AMSL)
        :type altitude_m: float
        """

        self._submit_coroutine(
            self._system.gimbal.set_roi_location(
                latitude_deg, longitude_deg, altitude_m
            )
        )

    def take_control(self, control_mode: ControlMode) -> None:
        """
        Take control. There can be only two components in control of a gimbal at any given time.

        :param control_mode: Control mode (primary or secondary)
        :type control_mode: ControlMode
        """

        self._submit_coroutine(self._system.gimbal.take_control(control_mode))

    def register_control_handler(self, handler: Callable) -> None:
        """
        Registers a function (Callable) to be a handler of the data stream
        :param handler: A Callable which gets executed each time new data is received
        """
        self._register_handler(self._system.gimbal.control)(handler)
