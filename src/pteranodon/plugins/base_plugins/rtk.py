from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from mavsdk.rtk import RtcmData


from .abstract_base_plugin import AbstractBasePlugin


class Rtk(AbstractBasePlugin):
    """
    Service to send RTK corrections to the vehicle.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("rtk", system, loop, logger)
        self._end_init()

    def send_rtcm_data(self, string_data: str) -> None:
        """
        Send RTCM data

        :param string_data: The data as a string to send
        :type string_data: str
        """
        rtcm_data = RtcmData(string_data)

        self._submit_coroutine(self._system.rtk.send_rtcm_data(rtcm_data))
