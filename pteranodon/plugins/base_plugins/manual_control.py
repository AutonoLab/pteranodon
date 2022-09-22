import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from .abstract_base_plugin import AbstractBasePlugin


class ManualControl(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("manual_control", system, loop, logger)

    def set_manual_control_input(self, x: float, y: float, z: float, r: float):

        """
        The manual contrl input needs to be sent at a rate high enough to prevent triggering of RC loss, a good
        minium rate is 10 Hz.
        Args:
            x: value between -1 to 1. negative -> backwards, positive -> forwards
            y: value between -1 to 1. negative -> left, positive -> right
            z: value between -1 to 1. negative -> down, positive -> up
            r: value between -1 to 1. negative -> turn anti-clockwise (towards the left), positive -> turn clockwise
            (towards the right)
        """
        super().submit_task(
            asyncio.ensure_future(
                self._system.manual_control.set_manual_control_input(x, y, z, r),
                loop=self._loop,
            )
        )

    def start_altitude_control(self):

        """
        Requires manual control input to be sent regularly already. Does not require a valid position ex: GPS
        """
        super().submit_task(
            asyncio.ensure_future(
                self._system.manual_control.start_altitude_control(), loop=self._loop
            )
        )

    def start_position_control(self):
        """
        Start position control using e.g. joystick input. Requires manual control input to be sent regularly already.
        Requires a valid position using e.g. GPS, external vision, or optical flow
        """
        super().submit_task(
            asyncio.ensure_future(
                self._system.manual_control.start_position_control(), loop=self._loop
            )
        )
