import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from time import sleep
from typing import List, Dict, Any, Callable, AsyncGenerator
from functools import partial

from mavsdk import System, action

from .abstract_base_plugin import AbstractBasePlugin


class Action(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("action", system, loop, logger)

        self._maximum_speed = None
        self._maximum_speed_task = asyncio.ensure_future(self._system.action.get_maximum_speed(), loop=self._loop)
        self._maximum_speed_task.add_done_callback(partial(self._maximum_speed_callback))

        self._launch_altitude = None
        self._launch_altitude_task = asyncio.ensure_future(self._system.action.get_return_to_launch_altitude(),
                                                           loop=self._loop)
        self._launch_altitude_task.add_done_callback(partial(self._launch_altitude_callback))

        self._takeoff_altitude = None
        self._takeoff_altitude_task = asyncio.ensure_future(self._system.action.get_takeoff_altitude(), loop=self._loop)
        self._takeoff_altitude_task.add_done_callback(partial(self._takeoff_altitude_callback))

    def _maximum_speed_callback(self, task: Task) -> None:
        self._maximum_speed = task.result()
        del self._maximum_speed_task

    def _launch_altitude_callback(self, task: Task) -> None:
        self._launch_altitude = task.result()
        del self._launch_altitude_task

    def _takeoff_altitude_callback(self, task: Task) -> None:
        self._takeoff_altitude = task.result()
        del self._takeoff_altitude_task

    def arm(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.arm(), loop=self._loop)
        )

    def disarm(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.disarm(), loop=self._loop)
        )

    def do_orbit(self, radius_m: float, velocity_ms: float, yaw_behavior: action.OrbitYawBehavior, latitude_deg: float,
                 longitude_deg: float, absolute_altitude_m: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.do_orbit(radius_m, velocity_ms, yaw_behavior, latitude_deg,
                                                               longitude_deg, absolute_altitude_m), loop=self._loop)
        )

    def get_maximum_speed(self) -> float:
        return self._maximum_speed

    def get_return_to_launch_altitude(self) -> float:
        return self._launch_altitude

    def get_takeoff_altitude(self) -> float:
        return self._takeoff_altitude

    def goto_location(self, latitude_deg: float, longitude_deg: float, absolute_altitude_m: float, yaw: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.goto_location(latitude_deg, longitude_deg,
                                                                    absolute_altitude_m, yaw), loop=self._loop)
        )

    def hold(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.hold(), loop=self._loop)
        )

    def kill(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.kill(), loop=self._loop)
        )

    def reboot(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.reboot(), loop=self._loop)
        )

    def return_to_launch(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.return_to_launch(), loop=self._loop)
        )

    def set_acuator(self, index: int, value: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.set_acuator(index, value), loop=self._loop)
        )

    def set_current_speed(self, speed_m_s: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.set_current_speed(speed_m_s), loop=self._loop)
        )

    def set_maximum_speed(self, speed_m_s: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.set_maximum_speed(speed_m_s), loop=self._loop)
        )
        self._maximum_speed = speed_m_s

    def set_return_to_launch_altitude(self, relative_altitude_m: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.set_return_to_launch_altitude(relative_altitude_m),
                                  loop=self._loop)
        )
        self._launch_altitude = relative_altitude_m

    def set_takeoff_altitude(self, relative_altitude_m: float) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.set_takeoff_altitude(relative_altitude_m), loop=self._loop)
        )
        self._takeoff_altitude = relative_altitude_m

    def shutdown(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.shutdown(), loop=self._loop)
        )

    def takeoff(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.takeoff(), loop=self._loop)
        )

    def terminate(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.terminate(), loop=self._loop)
        )

    def transition_to_fixedwing(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.transition_to_fixedwing(), loop=self._loop)
        )

    def transition_to_multicopter(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.action.transition_to_multicopter(), loop=self._loop)
        )
