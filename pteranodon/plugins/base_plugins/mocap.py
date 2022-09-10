import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger

from mavsdk import System, mocap
from mocap import AttitudePositionMocap
from mocap import Odometry
from mocap import VisionPositionEstimate

from .abstract_base_plugin import AbstractBasePlugin


class Mocap(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mocap", system, loop, logger)

    def set_attitude_position_mocap(self, attitude_position_mocap: AttitudePositionMocap) -> None:
        """
        Send motion capture attitude and position

        :param attitude_position_mocap: The attitude and position data
        :type attitude_position_mocap: AttitudePositionMocap
        """

        self._logger.info("Set Mocap attitude and position")
        super().submit_task(
            asyncio.ensure_future(self._system.mocap.set_attitude_position_mocap(attitude_position_mocap), loop=self._loop)
        )

    def set_odometry(self, odometry : Odometry) -> None:
        """
        Send odometry information with an external interface.

        :param odometry: The odometry data
        :type odemetry: Odometry
        """

        super().submit_task(
            asyncio.ensure_future(self._system.mocap.set_odometry(odometry))
        )

    def set_vision_position_estimate(self, vision_position_estimate : VisionPositionEstimate) -> None:
        """
        Send Global position/attitude estimate from a vision source.

        :param vision_position_estimate: The vision position estimate
        :type vision_position_estimate: VisionPositionEstimate
        """

        super().submit_task(
            asyncio.ensure_future(self._system.mocap.set_vision_position_estimate(vision_position_estimate))
        )