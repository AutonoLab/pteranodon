import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from time import sleep
from typing import List, Dict, Any, Callable

from mavsdk import System, follow_me


class FollowMe:
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        self._system = system
        self._loop = loop
        self._logger = logger

        # only gotta wait on async tasks to get the data non-async since we store the parameters in method calls
        tasks = []
        tasks.append(asyncio.ensure_future(self._system.follow_me.is_active(), loop=self._loop))
        tasks.append(asyncio.ensure_future(self._system.follow_me.get_last_location(), loop=self._loop))
        tasks.append(asyncio.ensure_future(self._system.follow_me.get_config(), loop=self._loop))
        for task in tasks:
            while not task.done():
                sleep(0.000001)

        self._is_active = tasks[0].result()
        self._last_location = tasks[1].result()
        self._config = tasks[2].result()

    def get_config(self) -> follow_me.Config:
        return self._config

    def get_last_location(self) -> follow_me.TargetLocation:
        return self._last_location

    def is_active(self) -> bool:
        return self._is_active

    def set_config(self, config: follow_me.Config) -> None:
        asyncio.ensure_future(self._system.follow_me.set_config(config), loop=self._loop)
        self._config = config

    def set_target_location(self, location: follow_me.TargetLocation) -> None:
        asyncio.ensure_future(self._system.follow_me.set_target_location(location), loop=self._loop)
        self._last_location = location

    def start(self) -> None:
        asyncio.ensure_future(self._system.follow_me.start(), loop=self._loop)
        self._is_active = True

    def stop(self) -> None:
        asyncio.ensure_future(self._system.follow_me.start(), loop=self._loop)
        self._is_active = False
