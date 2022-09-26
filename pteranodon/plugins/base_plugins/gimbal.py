import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from mavsdk.gimbal import GimbalMode, ControlMode, ControlStatus

from .abstract_base_plugin import AbstractBasePlugin


class Gimbal(AbstractBasePlugin):
    """
    Provide control over a gimbal.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("gimbal", system, loop, logger)

        self._control_status = None
        self._control_task = asyncio.ensure_future(
            self._update_control_state(), loop=self._loop
        )

    async def _update_control_state(self) -> None:
        """
        Subscribe to control status updates. This allows a component to know if it has primary, secondary or no control
         over the gimbal. Also, it gives the system and component ids of the other components in control (if any).
        """

        async for ctrl_status in self._system.gimbal.receive():
            if ctrl_status != self._control_status:
                self._control_status = ctrl_status

    def control(self) -> ControlStatus:
        """
        The current control status.

        :return: the current control status
        """
        return self._control_status

    def release_control(self) -> None:
        """
        Release control, such that other components can control the gimbal.
        """

        super().submit_task(
            asyncio.ensure_future(self._system.gimbal.release_control())
        )

    def set_mode(self, gimbal_mode: GimbalMode) -> None:
        """
        Sets the desired yaw mode of a gimbal.

        :param gimbal_mode: The mode to be set
        :type gimbal_mode: GimbalMode
        """

        super().submit_task(
            asyncio.ensure_future(self._system.gimbal.set_mode(gimbal_mode))
        )

    def set_pitch_and_yaw(self, pitch_deg: float, yaw_deg: float) -> None:
        """
        Set gimbal pitch and yaw angles.

        :param pitch_deg: Pitch angle in degrees (negative points down)
        :type pitch_deg: float

        :param yaw_deg: Yaw angle in degrees (positive is clock-wise, range: -180 to 180 or 0 to 360)
        :type yaw_deg: float
        """

        super().submit_task(
            asyncio.ensure_future(
                self._system.gimbal.set_pitch_and_yaw(pitch_deg, yaw_deg)
            )
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

        super().submit_task(
            asyncio.ensure_future(
                self._system.gimbal.set_pitch_rate_and_yaw_rate(
                    pitch_rate_deg_s, yaw_rate_deg_s
                )
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

        super().submit_task(
            asyncio.ensure_future(
                self._system.gimbal.set_roi_location(
                    latitude_deg, longitude_deg, altitude_m
                )
            )
        )

    def take_control(self, control_mode: ControlMode) -> None:
        """
        Take control. There can be only two components in control of a gimbal at any given time.

        :param control_mode: Control mode (primary or secondary)
        :type control_mode: ControlMode
        """

        super().submit_task(
            asyncio.ensure_future(self._system.gimbal.set_roi_location(control_mode))
        )
