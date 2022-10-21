from asyncio import AbstractEventLoop
from logging import Logger
from functools import partial

from mavsdk import System, transponder

from .abstract_base_plugin import AbstractBasePlugin


class Transponder(AbstractBasePlugin):
    """
    Allow users to get ADS-B information and set ADS-B update rates.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("transponder", system, loop, logger)

        self._submit_simple_generator(self._system.transponder.transponder)
        self.register_transponder_handler = partial(
            self._register_handler, self._system.transponder.transponder
        )

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
