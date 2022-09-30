from abc import ABC
import asyncio
from asyncio import AbstractEventLoop, coroutine
from concurrent.futures import Future
from logging import Logger
from collections import deque
from functools import partial
from typing import Tuple, Any

from mavsdk import System


class AbstractPlugin(ABC):
    """
    Base plugin functionality, no methods required to overwrite
    """

    def __init__(
        self, name: str, system: System, loop: AbstractEventLoop, logger: Logger
    ) -> None:
        self._name: str = name
        self._system: System = system
        self._loop: AbstractEventLoop = loop
        self._logger: Logger = logger

        self._future_cache: deque = deque()
        self._name_cache: deque = deque()
        self._result_cache: deque[Tuple[str, Any]] = deque(maxlen=10)

    @property
    def name(self) -> str:
        """
        :return: str ; returns the name of the plugin as a string
        """
        return self._name

    def _future_callback(self, future: Future) -> None:
        coro_name: str = self._name_cache[self._future_cache.index(future)]
        self._logger.info(f"Task completed: {coro_name} ")
        try:
            self._result_cache.append((coro_name, future.result()))
        except Exception as e:
            self._logger.error(e)
        self._future_cache.remove(future)
        self._name_cache.remove(coro_name)

    def _submit_coroutine(self, coro: coroutine) -> Future:
        """
        Puts a task returned by asyncio.run_coroutine_threadsafe to the future_cache to prevent garbage collection and allow return
        value analysis
        :param new_task: A concurrent.future.Future
        :return: The submitted task, if plugin specific callbacks are added
        """
        new_future = asyncio.run_coroutine_threadsafe(coro, loop=self._loop)
        new_future.add_done_callback(partial(self._future_callback))
        self._future_cache.append(new_future)
        self._name_cache.append(coro.__qualname__)  # type: ignore
        return new_future
