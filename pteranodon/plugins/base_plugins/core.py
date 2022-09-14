import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, core

from .abstract_base_plugin import AbstractBasePlugin


class Core(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("core", system, loop, logger)

        self._connection_state = None
        self._connection_task = asyncio.ensure_future(self._update_connection_state(), loop=self._loop)

    def set_mavlink_timeout(self, delay_s: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.core.set_mavlink_timeout(delay_s), loop=self._loop)
        )

    async def _update_connection_state(self) -> None:
        async for connection_state in self._system.core.connection_state():
            if connection_state != self._connection_state:
                self._connection_state = connection_state

    def connection_state(self) -> core.ConnectionState:
        return self._connection_state
