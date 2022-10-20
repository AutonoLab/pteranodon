from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, core

from .abstract_base_plugin import AbstractBasePlugin


class Core(AbstractBasePlugin):
    """
    Access to the connection state and core configurations
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("core", system, loop, logger)

        self._submit_simple_generator(self._system.core.connection_state)

        self._end_init()

    def set_mavlink_timeout(self, delay_s: float) -> None:
        """
        Set timeout of MAVLink transfers.

        The default timeout used is generally (0.5 seconds) seconds. If MAVSDK is used on the same host this timeout can
        be reduced, while if MAVSDK has to communicate over links with high latency it might need to be increased to
        prevent timeouts.
        :param delay_s: Timeout in seconds
        :return: None
        """
        self._submit_coroutine(self._system.core.set_mavlink_timeout(delay_s))

    def connection_state(self) -> core.ConnectionState:
        """
        Subscribe to 'connection state' updates
        :return: core.ConnectionState ; The current connection state
        """
        return self._async_gen_data[self._system.core.connection_state]
