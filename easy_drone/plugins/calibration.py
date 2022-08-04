import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from time import sleep
from typing import List, Dict, Any, Callable

from mavsdk import System, calibration

from ..abstract_plugin import AbstractPlugin


class Calibration(AbstractPlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__(system, loop, logger)

    async def _calibrate_wrapper(self, com: Callable) -> None:
        sensor_name = com.__name__.split("_")[1]
        self._logger.info(f"Beginning calibration of {sensor_name}")
        async for data in com():
            self._logger.info(data)
        self._logger.info(f"Finished calibration of {sensor_name}")

    def calibrate_gyro(self) -> None:
        asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_gyro()), loop=self._loop)

    def calibrate_accelerometer(self) -> None:
        asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_accelerometer()),
                              loop=self._loop)

    def calibrate_gimbal_accelerometer(self) -> None:
        asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_gimbal_accelerometer()),
                              loop=self._loop)

    def calibrate_magnetometer(self) -> None:
        asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_magnetometer()),
                              loop=self._loop)

    def calibrate_level_horizon(self) -> None:
        asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_level_horizon()),
                              loop=self._loop)

    def cancel(self) -> None:
        asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.cancel()), loop=self._loop)

    def calibrate_all(self) -> None:
        self.calibrate_gyro()
        self.calibrate_accelerometer()
        self.calibrate_gimbal_accelerometer()
        self.calibrate_magnetometer()
        self.calibrate_level_horizon()
