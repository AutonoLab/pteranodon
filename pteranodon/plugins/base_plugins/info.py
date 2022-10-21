import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, info

from .abstract_base_plugin import AbstractBasePlugin


class Info(AbstractBasePlugin):
    """
    Provide information about the hardware and/or software of a system.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("info", system, loop, logger)

        self._id: Optional[info.Identification] = None
        self._product: Optional[info.Product] = None
        self._version: Optional[info.Version] = None
        self._flight_info: Optional[info.FlightInfo] = None
        self._speed_factor: Optional[float] = None

        self._acquire_attempts: int = 1
        self._loop.run_until_complete(self._get_id())
        self._loop.run_until_complete(self._get_product())
        self._loop.run_until_complete(self._get_version())
        self._loop.run_until_complete(self._flight_info_gen())
        self._loop.run_until_complete(self._speed_factor_gen())

        self._end_init()

    async def _attempt_acquire(self, info_coro):
        for _ in range(self._acquire_attempts):
            try:
                return await info_coro()
            except info.InfoError:
                pass

    async def _get_id(self) -> None:
        self._id = await self._attempt_acquire(self._system.info.get_identification)

    async def _get_product(self) -> None:
        self._product = await self._attempt_acquire(self._system.info.get_product)


    async def _get_version(self) -> None:
        self._version = await self._attempt_acquire(self._system.info.get_version)

    async def _flight_info_gen(self) -> None:
        self._flight_info = await self._attempt_acquire(self._system.info.get_flight_information)

    async def _speed_factor_gen(self) -> None:
        self._speed_factor = await self._attempt_acquire(self._system.info.get_speed_factor)

    def get_identification(self) -> Optional[info.Identification]:
        """
        :return: info.Identification ; Returns the uuid or identification of the hardware system
        """
        return self._id

    def get_product(self) -> Optional[info.Product]:
        """
        :return: info.Product ; returns system product information
        """
        return self._product

    def get_version(self) -> Optional[info.Version]:
        """
        :return: info.Version ; returns system software information
        """
        return self._version

    def get_flight_information(self) -> Optional[info.FlightInfo]:
        """
        :return: info.FlightInfo ; returns system flight information
        """
        return self._flight_info

    def get_speed_factor(self) -> Optional[float]:
        """
        :return: float ; Returns the speed factor of simulation
        """
        return self._speed_factor
