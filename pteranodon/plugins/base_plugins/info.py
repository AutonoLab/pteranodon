import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, info

from .abstract_base_plugin import AbstractBasePlugin


class Info(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("info", system, loop, logger)

        self._id: Optional[info.Identification] = None
        self._product: Optional[info.Product] = None
        self._version: Optional[info.Version] = None
        self._flight_info: Optional[info.FlightInfo] = None
        self._speed_factor: Optional[float] = None

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
        try:
            while True:
                self._flight_info = await self._system.info.get_flight_information()
                await asyncio.sleep(self._flight_info_rate)
        except info.InfoError as e:
            self._logger.error(e)

    async def _speed_factor_gen(self) -> None:
        try:
            while True:
                self._speed_factor = await self._system.info.get_speed_factor()
                await asyncio.sleep(self._speed_factor_rate)
        except info.InfoError as e:
            self._logger.error(e)

    def get_identification(self) -> Optional[info.Identification]:
        return self._id

    def get_product(self) -> Optional[info.Product]:
        return self._product

    def get_version(self) -> Optional[info.Version]:
        return self._version

    def get_flight_information(self) -> Optional[info.FlightInfo]:
        return self._flight_info

    def get_speed_factor(self) -> Optional[float]:
        return self._speed_factor

    def set_flight_information_rate(self, rate: float) -> None:
        self._flight_info_rate = rate

    def set_speed_factor_rate(self, rate: float) -> None:
        self._speed_factor_rate = rate
