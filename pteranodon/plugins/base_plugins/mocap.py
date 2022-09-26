import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, mocap

from .abstract_base_plugin import AbstractBasePlugin


class Mocap(AbstractBasePlugin):
    """
    Allows interfacing a vehicle with a motion capture system in order to allow navigation without
    global positioning sources available (e.g. indoors, or when flying under a bridge. etc.).
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mocap", system, loop, logger)

    def set_attitude_position_mocap(
        self, attitude_position_mocap: mocap.AttitudePositionMocap
    ) -> None:
        """
        Send motion capture attitude and position

        :param attitude_position_mocap: The attitude and position data
        :type attitude_position_mocap: mocap.AttitudePositionMocap
        """

        self._logger.info("Set Mocap attitude and position")
        super().submit_task(
            asyncio.ensure_future(
                self._system.mocap.set_attitude_position_mocap(attitude_position_mocap),
                loop=self._loop,
            )
        )

    def set_odometry(self, odometry: mocap.Odometry) -> None:
        """
        Send odometry information with an external interface.

        :param odometry: The odometry data
        :type odemetry: mocap.Odometry
        """

        super().submit_task(
            asyncio.ensure_future(
                self._system.mocap.set_odometry(odometry), loop=self._loop
            )
        )

    def set_vision_position_estimate(
        self, vision_position_estimate: mocap.VisionPositionEstimate
    ) -> None:
        """
        Send Global position/attitude estimate from a vision source.

        :param vision_position_estimate: The vision position estimate
        :type vision_position_estimate: mocap.VisionPositionEstimate
        """

        super().submit_task(
            asyncio.ensure_future(
                self._system.mocap.set_vision_position_estimate(
                    vision_position_estimate
                ),
                loop=self._loop,
            )
        )
