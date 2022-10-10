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

        self._flight_info_rate: float = 1.0 / 2.0
        self._speed_factor_rate: float = 1.0 / 2.0

        self._acquire_attempts: int = 1
        self._loop.run_until_complete(self._get_id())
        self._loop.run_until_complete(self._get_product())
        self._loop.run_until_complete(self._get_version())
        self._submit_generator(self._flight_info_gen)
        self._submit_generator(self._speed_factor_gen)

        self._end_init()

    async def _get_id(self) -> None:
        for _ in range(self._acquire_attempts):
            try:
                self._id = await self._system.info.get_identification()
                break
            except info.InfoError:
                pass

    async def _get_product(self) -> None:
        for _ in range(self._acquire_attempts):
            try:
                self._product = await self._system.info.get_product()
                break
            except info.InfoError:
                pass

    async def _get_version(self) -> None:
        for _ in range(self._acquire_attempts):
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

    def set_flight_information_rate(self, rate: float) -> None:
        """
        :param rate: float ; The desired rate of information updates per second
        :return: None
        """
        self._flight_info_rate = 1.0 / rate

    def set_speed_factor_rate(self, rate: float) -> None:
        """
        :param rate: float ; Sets the speed factor of simulation, simulations can run tasks faster than real time
        :return: None
        """
        self._speed_factor_rate = 1.0 / rate
