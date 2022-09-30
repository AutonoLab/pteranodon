import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, transponder

from .abstract_base_plugin import AbstractBasePlugin


class Transponder(AbstractBasePlugin):
    """
    Allow users to get ADS-B information and set ADS-B update rates.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("transponder", system, loop, logger)

        self._transponder_data = None
        self._transponder_task = asyncio.run_coroutine_threadsafe(
            self._transponder_update(), loop=self._loop
        )

    def set_rate_transponder(self, rate: float) -> None:
        """
        Set rate of transponder updates
        :param rate: Requested rate in Hz
        :return: None
        """
        self._submit_coroutine(self._system.transponder.set_rate_transponder(rate))

    async def _transponder_update(self) -> None:
        async for transponder_val in self._system.transponder.transponder():
            if transponder_val != self._transponder_data:
                self._transponder_data = transponder_val

    def transponder(self) -> transponder.AdsbVehicle:
        """
        Subscribe to transponder updates
        :return: transponder.AdsbVehicle ; The next transponder detection
        """
        return self._transponder_data
