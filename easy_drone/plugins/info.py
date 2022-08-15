import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from time import sleep
from typing import List, Dict, Any, Callable

from mavsdk import System, info

from ..abstract_plugin import AbstractPlugin


class Info(AbstractPlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__(system, loop, logger)

        self._id = None
        self._product = None
        self._version = None
        self._flight_info = None
        self._speed_factor = None

        self._flight_info_rate = 2.0
        self._speed_factor_rate = 2.0

        self._id_task = asyncio.ensure_future(self._get_id(), loop=self._loop)
        self._product_task = asyncio.ensure_future(self._get_product(), loop=self._loop)
        self._version_task = asyncio.ensure_future(self._get_version(), loop=self._loop)
        self._flight_info_task = asyncio.ensure_future(self._flight_info_gen(), loop=self._loop)
        self._speed_factor_task = asyncio.ensure_future(self._speed_factor_gen(), loop=self._loop)

    async def _get_id(self) -> None:
        while True:
            try:
                self._id = await self._system.info.get_identification()
                break                
            except info.InfoError:
                pass
    
    async def _get_product(self) -> None:
        while True:
            try:
                self._product = await self._system.info.get_product()
                break                
            except info.InfoError:
                pass
    
    async def _get_version(self) -> None:
        while True:
            try:
                self._version = await self._system.info.get_version()
                break                
            except info.InfoError:
                pass

    async def _flight_info_gen(self) -> None:
        while True:
            self._flight_info = await self._system.info.get_flight_information()
            await asyncio.sleep(self._flight_info_rate)

    async def _speed_factor_gen(self) -> None:
        while True:
            self._speed_factor = await self._system.info.get_speed_factor()
            await asyncio.sleep(self._speed_factor_rate)

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

    def set_flight_information_rate(self, rate: float) -> None:
        self._flight_info_rate = rate

    def set_speed_factor_rate(self, rate: float) -> None:
        self._speed_factor_rate = rate
