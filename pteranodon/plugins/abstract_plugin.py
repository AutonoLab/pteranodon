from abc import ABC
import asyncio
from asyncio import AbstractEventLoop, coroutine
from concurrent.futures import Future
from logging import Logger
from collections import deque
from functools import partial
from typing import Tuple, Any, Callable, Optional

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
        """
        Callback associated with each Future scheduled by method _submit_coroutine.
        This will send output to the logger, retrieve results and exceptions, and clear Futures from the cache.
        :param future: A concurrent.future.Future, which has been scheduled with asyncio.run_coroutine_threadsafe
        :return: None
        """
        try:
            coro_name: str = self._name_cache[self._future_cache.index(future)]
            self._logger.info(f"Task completed: {coro_name} ")
            try:
                self._result_cache.append((coro_name, future.result()))
            except Exception as e:
                self._logger.error(f"{coro_name} -> {e}")
            self._future_cache.remove(future)
            self._name_cache.remove(coro_name)
        except Exception as e:
            self._logger.error(f"Callback failed: {future}")

    def _submit_coroutine(
        self, coro: coroutine, callback: Optional[Callable] = None
    ) -> Future:
        """
        Puts a task returned by asyncio.run_coroutine_threadsafe to the future_cache to prevent garbage collection and allow return
        value analysis
        :param coro: A concurrent.future.Future
        :param callback: A Callable, which will be added with add_done_callback to the given coroutine
        :return: The submitted task, if plugin specific callbacks are added
        """
        new_future = asyncio.run_coroutine_threadsafe(coro, loop=self._loop)
        new_future.add_done_callback(partial(self._future_callback))
        if callback is not None:
            new_future.add_done_callback(callback)
        self._future_cache.append(new_future)
        self._name_cache.append(coro.__qualname__)
        return new_future

    def _schedule(self, *args: coroutine) -> None:
        """
        Takes any ammount of coroutins as input and uses add_done_callback to chain all coroutines together
        :param *args: Any amount of coroutines
        :return: The scheduled Future with coroutines chained to it
        """
        coros = [*args]
        if len(coros) > 0:
            if isinstance(
                coros[-1], Future
            ):  # remove the Future if this was called with a callback
                coros.pop()
            first = coros[0]
            coros.remove(first)
            self._submit_coroutine(first, partial(self._schedule, *coros))
