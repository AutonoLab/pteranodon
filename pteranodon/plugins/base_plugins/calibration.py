import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import AsyncGenerator

from mavsdk import System, calibration

from .abstract_base_plugin import AbstractBasePlugin


class Calibration(AbstractBasePlugin):
    """
    Enable to calibrate sensors of a drone such as gyro, accelerometer, and magnetometer.
    """
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("calibration", system, loop, logger)

    async def _calibrate_wrapper(self, com: AsyncGenerator) -> None:
        sensor_name = com.__name__.split("_")[1]  # type: ignore
        self._logger.info(f"Beginning calibration of {sensor_name}")
        try:
            async for _ in com:
                pass
        except calibration.CalibrationError as e:
            self._logger.error(f"{sensor_name} calibration {e}")
        self._logger.info(f"Finished calibration of {sensor_name}")

    def _calibrate_gyro(self) -> Task:
        return super().submit_task(
            asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_gyro()), loop=self._loop)
        )
    
    def calibrate_gyro(self) -> None:
        self._calibrate_gyro()

    def _calibrate_accelerometer(self) -> Task:
        return super().submit_task(
            asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_accelerometer()),
                                  loop=self._loop)
        )

    def calibrate_accelerometer(self) -> None:
        self._calibrate_accelerometer()

    def _calibrate_gimbal_accelerometer(self) -> Task:
        return super().submit_task(
            asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_gimbal_accelerometer()),
                                  loop=self._loop)
        )
    
    def calibrate_gimbal_accelerometer(self) -> None:
        self._calibrate_gimbal_accelerometer()

    def _calibrate_magnetometer(self) -> Task:
        return super().submit_task(
            asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_magnetometer()),
                                  loop=self._loop)
        )
    
    def calibrate_magnetometer(self) -> None:
        self._calibrate_magnetometer()

    def _calibrate_level_horizon(self) -> Task:
        return super().submit_task(
            asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.calibrate_level_horizon()),
                                  loop=self._loop)
        )
    
    def calibrate_level_horizon(self) -> None:
        self._calibrate_level_horizon()

    def cancel(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._calibrate_wrapper(self._system.calibration.cancel()), loop=self._loop)
        )

    async def _calibrate_all(self) -> None:
        task_funcs = [self._calibrate_gyro, self._calibrate_accelerometer, self._calibrate_gimbal_accelerometer,
                 self._calibrate_magnetometer, self._calibrate_level_horizon]
        for func in task_funcs:
            task = asyncio.ensure_future(func(), loop=self._loop)
            while not task.done():
                await asyncio.sleep(0.05)

    def calibrate_all(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._calibrate_all(), loop=self._loop)
        )
