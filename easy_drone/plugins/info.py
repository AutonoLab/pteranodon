import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from time import sleep
from typing import List, Dict, Any, Callable

from mavsdk import System, info

from ..abstract_plugin import AbstractPlugin


class Info(AbstractPlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__(system, loop, logger)

        self._id = self._loop.run_until_complete(self._system.info.get_identification())
        self._product = self._loop.run_until_complete(self._system.info.get_product())
        self._version = self._loop.run_until_complete(self._system.info.get_version())
        self._flight_info = None
        self._speed_factor = None

        self._flight_info_task = asyncio.ensure_future(self._make_async_gen(self._system.info.get_flight_information()),
                                                       loop=self._loop)
        self._speed_factor_task = asyncio.ensure_future(self._make_async_gen(self._system.info.get_speed_factor()),
                                                        loop=self._loop)

    async def _flight_info_gen(self) -> None:
        while True:
            self._flight_info = await self._system.info.get_flight_information()
            await asyncio.sleep(0.5)

    async def _speed_factor_gen(self) -> None:
        while True:
            self._speed_factor = await self._system.info.get_speed_factor()
            await asyncio.sleep(0.5)

    def get_identification(self) -> info.Identification:
        return self._id

    def get_product(self) -> info.Product:
        return self._product

    def get_version(self) -> info.Version:
        return self._version

    def get_flight_information(self) -> info.FlightInfo:
        return self._flight_info

    def get_speed_factor(self) -> float:
        return self._speed_factor
