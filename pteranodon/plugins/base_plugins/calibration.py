import asyncio
from asyncio import AbstractEventLoop, Future
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

    def _calibrate_gyro(self) -> Future:
        return super().submit_coroutine(
            self._calibrate_wrapper(self._system.calibration.calibrate_gyro())
        )

    def calibrate_gyro(self) -> None:
        """
        Perform gyro calibration
        :return: None
        """
        self._calibrate_gyro()

    def _calibrate_accelerometer(self) -> Future:
        return super().submit_coroutine(
            self._calibrate_wrapper(self._system.calibration.calibrate_accelerometer())
        )

    def calibrate_accelerometer(self) -> None:
        """
        Perform accelerometer calibration
        :return: None
        """
        self._calibrate_accelerometer()

    def _calibrate_gimbal_accelerometer(self) -> Future:
        return super().submit_coroutine(
            self._calibrate_wrapper(
                self._system.calibration.calibrate_gimbal_accelerometer()
            )
        )

    def calibrate_gimbal_accelerometer(self) -> None:
        """
        Perform gimbal accelerometer calibration.
        :return: None
        """
        self._calibrate_gimbal_accelerometer()

    def _calibrate_magnetometer(self) -> Future:
        return super().submit_coroutine(
            self._calibrate_wrapper(self._system.calibration.calibrate_magnetometer())
        )

    def calibrate_magnetometer(self) -> None:
        """
        Perform magnetometer calibration.
        :return: None
        """
        self._calibrate_magnetometer()

    def _calibrate_level_horizon(self) -> Future:
        return super().submit_coroutine(
            self._calibrate_wrapper(self._system.calibration.calibrate_level_horizon())
        )

    def calibrate_level_horizon(self) -> None:
        """
        Perform board level horizon calibration.
        :return:
        """
        self._calibrate_level_horizon()

    def cancel(self) -> None:
        """
        Cancel ongoing calibration process.
        :return:
        """

        super().submit_coroutine(self._system.calibration.cancel())

    async def _calibrate_all(self) -> None:
        task_funcs = [
            self._calibrate_gyro,
            self._calibrate_accelerometer,
            self._calibrate_gimbal_accelerometer,
            self._calibrate_magnetometer,
            self._calibrate_level_horizon,
        ]
        for func in task_funcs:
            task = asyncio.ensure_future(func(), loop=self._loop)
            while not task.done():
                await asyncio.sleep(0.05)

    def calibrate_all(self) -> None:
        """
        Perform calibrations on all available types of sensor.
        :return: None
        """
        super().submit_coroutine(self._calibrate_all())
