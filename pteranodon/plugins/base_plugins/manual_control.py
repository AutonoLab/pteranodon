import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
#from mavsdk.manual_control

from .abstract_base_plugin import AbstractBasePlugin

class ManualControl(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("manual_control", system, loop, logger)

    def set_manual_control_input(self, x: float, y: float, z: float, r: float):
        """
        Set manual control input
        The manual control input needs to be sent at rate high enough to prevent triggers of RC loss,
        a good minimum rate is 10 Hz.
        """
        super().submit_task(
            asyncio.ensure_future(self._system.manual_control.set_manual_control_input(x, y, z, r), loop=self._loop)
        )

    def start_altitude_control(self):
        """
        Requires manual control input to be sent regularly already. Does not require a valid position ex: GPS
        """
        super().submit_task(
            asyncio.ensure_future(self._system.manual_control.start_altitude_control(), loop=self._loop)
        )

    def start_position_control(self):
        """
        Start position control using e.g. joystick input.
        Requires manual control input to be sent regularly already. Requires a valid position using e.g. GPS, external vision, or optical flow
        """
        super().submit_task(
            asyncio.ensure_future(self._system.manual_control.start_position_control(), loop=self._loop)
        )
