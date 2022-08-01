import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from time import sleep
from typing import Callable

from mavsdk import System
from mavsdk import param


class Param:
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        self._system = system
        self._loop = loop
        self._logger = logger

    def _get_async_data(self, com: Callable, *args, **kwargs) -> Any:
        task = asyncio.ensure_future(com(*args, **kwargs), loop=self._loop)
        while not task.done():
            sleep(0.000001)
        return task.result()

    def get_all_params(self) -> param.AllParams:
        return self._get_async_data(self._system.param.get_all_params)

    def get_param_custom(self, name: str) -> str:
        return self._get_async_data(self._system.param.get_param_custom, name)

    def get_param_float(self, name: str) -> float:
        return self._get_async_data(self._system.param.get_param_float, name)

    def get_param_int(self, name: str) -> int:
        return self._get_async_data(self._system.param.get_param_int, name)

    def set_param_custom(self, name: str, value: str) -> None:
        asyncio.ensure_future(self._system.param.set_param_custom(name, value), loop=self._loop)

    def set_param_custom(self, name: str, value: float) -> None:
        asyncio.ensure_future(self._system.param.set_param_float(name, value), loop=self._loop)

    def set_param_custom(self, name: str, value: int) -> None:
        asyncio.ensure_future(self._system.param.set_param_int(name, value), loop=self._loop)
