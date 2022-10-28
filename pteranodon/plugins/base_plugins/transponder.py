from asyncio import AbstractEventLoop
from logging import Logger
from typing import Callable

from mavsdk import System, transponder

from .abstract_base_plugin import AbstractBasePlugin


class Transponder(AbstractBasePlugin):
    """
    Allow users to get ADS-B information and set ADS-B update rates.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("transponder", system, loop, logger)

        self._submit_simple_generator(self._system.transponder.transponder)

        self._end_init()

    def set_rate_transponder(self, rate: float) -> None:
        """
        Set rate of transponder updates
        :param rate: Requested rate in Hz
        :return: None
        """
        self._submit_coroutine(self._system.transponder.set_rate_transponder(rate))

    def transponder(self) -> transponder.AdsbVehicle:
        """
        Subscribe to transponder updates
        :return: transponder.AdsbVehicle ; The next transponder detection
        """
        return self._async_gen_data[self._system.transponder.transponder]

    def register_transponder_handler(self, handler: Callable) -> None:
        """
        Registers a function (Callable) to be a handler of the data stream
        :param handler: A Callable which gets executed each time new data is received
        """
        self._register_handler(self._system.transponder.transponder)(handler)
