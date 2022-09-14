import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from functools import partial

from mavsdk import System, follow_me

from .abstract_base_plugin import AbstractBasePlugin


class FollowMe(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("follow_me", system, loop, logger)
        self._is_active = None
        self._last_location = None
        self._config = None

        # only gotta wait on async tasks to get the data non-async since we store the parameters in method calls
        self._is_active_task = asyncio.ensure_future(self._system.follow_me.is_active(), loop=self._loop)
        self._is_active_task.add_done_callback(partial(self._is_active_callback))
        self._last_location_task = asyncio.ensure_future(self._system.follow_me.get_last_location(), loop=self._loop)
        self._last_location_task.add_done_callback(partial(self._last_location_callback))
        self._config_task = asyncio.ensure_future(self._system.follow_me.get_config(), loop=self._loop)
        self._config_task.add_done_callback(partial(self._config_callback))

    def _is_active_callback(self, task: Task) -> None:
        self._is_active = task.result()
        del self._is_active_task

    def _last_location_callback(self, task: Task) -> None:
        self._last_location = task.result()
        del self._last_location_task

    def _config_callback(self, task: Task) -> None:
        self._config = task.result()
        del self._config_task

    def get_config(self) -> follow_me.Config:
        return self._config

    def get_last_location(self) -> follow_me.TargetLocation:
        return self._last_location

    def is_active(self) -> bool:
        return self._is_active

    def set_config(self, config: follow_me.Config) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.follow_me.set_config(config), loop=self._loop)
        )
        self._config = config

    def set_target_location(self, location: follow_me.TargetLocation) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.follow_me.set_target_location(location), loop=self._loop)
        )
        self._last_location = location

    def start(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.follow_me.start(), loop=self._loop)
        )
        self._is_active = True

    def stop(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.follow_me.start(), loop=self._loop)
        )
        self._is_active = False
