import asyncio
import typing
from abc import ABC
from asyncio import AbstractEventLoop, Task, Future
from logging import Logger
from collections import deque
from functools import partial
import platform
from typing import Callable

from mavsdk import System


class AbstractPlugin(ABC):
    """
    Base plugin functionality, no methods required to overwrite
    """

    def __init__(
        self, name: str, system: System, loop: AbstractEventLoop, logger: Logger
    ) -> None:
        self._name = name
        self._system = system
        self._loop = loop
        self._logger = logger

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

    def _task_callback(self, coroutine_name, task: Future) -> None:
        if self._use_coro_names:
            self._logger.info(f"Task completed: {coroutine_name} ")  # type: ignore
        try:
            self._result_cache.append(task.result())
        except Exception as e:
            self._logger.error(e)


    def submit_coroutine(self, new_coroutine : typing.Coroutine) -> Future:
        future = self.submit_task(
            asyncio.run_coroutine_threadsafe(new_coroutine, self._loop),
            coroutine_name=new_coroutine.__qualname__
        )
        return future

    def submit_task(self, new_task: Future, coroutine_name : str = "Unknown Coroutine") -> Future:
        """
        Puts a task returned by asyncio.ensure_future to the task_cache to prevent garbage collection and allow return
        value analysis
        :param new_task: An asyncio.Task
        :return: The submitted task, if plugin specific callbacks are added
        """
        new_task.add_done_callback(partial(self._task_callback, coroutine_name))
        self._task_cache.append(new_task)
        return new_task
