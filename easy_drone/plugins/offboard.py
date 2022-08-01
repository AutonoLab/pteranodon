import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from time import sleep
from typing import Callable

from mavsdk import System
from mavsdk import offboard
import mavsdk.offboard as offboard


class Offboard:
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        self._system = system
        self._loop = loop
        self._logger = logger

    def _get_async_data(self, com: Callable, *args, **kwargs) -> Any:
        task = asyncio.ensure_future(com(*args, **kwargs), loop=self._loop)
        while not task.done():
            sleep(0.000001)
        return task.result()

    def is_active(self) -> bool:
        return self._get_async_data(self._system.offboard.is_active)

    def set_acceleration_ned(self, accel_ned: offboard.AccelerationNed) -> None:
        asyncio.ensure_future(self._system.offboard.set_acceleration_ned(accel_ned), loop=self._loop)

    def set_acuator_control(self, act_ctrl: offboard.AcuatorControl) -> None:
        asyncio.ensure_future(self._system.offboard.set_acuator_control(act_ctrl), loop=self._loop)

    def set_attitude(self, attitude: offboard.Attitude) -> None:
        asyncio.ensure_future(self._system.offboard.set_attitude(attitude), loop=self._loop)

    def set_attitude_rate(self, attitude_rate: offboard.AttitudeRate) -> None:
        asyncio.ensure_future(self._system.offboard.set_attitude_rate(attitude_rate), loop=self._loop)

    def set_position_global(self, pos_global: offboard.PositionGlobalYaw) -> None:
        asyncio.ensure_future(self._system.offboard.set_position_global(pos_global), loop=self._loop)

    def set_position_ned(self, pos_ned: offboard.PositionNedYaw) -> None:
        asyncio.ensure_future(self._system.offboard.set_position_ned(pos_ned), loop=self._loop)

    def set_position_velocity_ned(self, pos: offboard.PositionNedYaw, vel: offboard.VelocityNedYaw) -> None:
        asyncio.ensure_future(self._system.offboard.set_position_velocity_ned(pos, vel), loop=self._loop)

    def set_velocity_body(self, vel_body: offboard.VelocityBodyYawSpeed) -> None:
        asyncio.ensure_future(self._system.offboard.set_velocity_body(vel_body), loop=self._loop)

    def set_velocity_ned(self, vel_ned: offboard.VelocityNedYaw) -> None:
        asyncio.ensure_future(self._system.offboard.set_velocity_ned(vel_ned), loop=self._loop)

    def start(self) -> None:
        asyncio.ensure_future(self._system.offboard.start(), loop=self._loop)

    def stop(self) -> None:
        asyncio.ensure_future(self._system.offboard.stop(), loop=self._loop)
