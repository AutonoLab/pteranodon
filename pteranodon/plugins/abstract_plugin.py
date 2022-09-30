from abc import ABC
import asyncio
from asyncio import AbstractEventLoop
from concurrent.futures import Future
from logging import Logger
from collections import deque
from functools import partial
import platform

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

        self._use_coro_names = True
        if int(platform.python_version().split(".")[1]) < 8:
            self._use_coro_names = False

        self._task_cache: deque = deque(maxlen=10)
        self._result_cache: deque = deque(maxlen=10)

    @property
    def name(self) -> str:
        """
        :return: str ; returns the name of the plugin as a string
        """
        return self._name

    def _task_callback(self, future: Future) -> None:
        if self._use_coro_names:
            # self._logger.info(f"Task completed: {task.get_coro().__qualname__} ")  # type: ignore
            # TODO, somehow attatch a name
            pass
        try:
            self._result_cache.append(future.result())
        except Exception as e:
            self._logger.error(e)

    # TODO, update the typing on this
    # TODO, update so it takes the coroutine and runs asyncio.run_coroutine_threadsafe for you
    def _submit_coroutine(self, new_funcall) -> Future:
        """
        Puts a task returned by asyncio.run_coroutine_threadsafe to the task_cache to prevent garbage collection and allow return
        value analysis
        :param new_task: A concurrent.future.Future
        :return: The submitted task, if plugin specific callbacks are added
        """
        new_future = asyncio.run_coroutine_threadsafe(new_funcall, loop=self._loop)
        new_future.add_done_callback(partial(self._task_callback))
        self._task_cache.append(new_future)
        return new_future
