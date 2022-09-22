from abc import ABC
from asyncio import AbstractEventLoop, Task
from logging import Logger
from collections import deque
from functools import partial
import platform

from mavsdk import System


class AbstractPlugin(ABC):
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
        return self._name

    def _task_callback(self, task: Task) -> None:
        if self._use_coro_names:
            self._logger.info(f"Task completed: {task.get_coro().__qualname__} ")  # type: ignore
        try:
            self._result_cache.append(task.result())
        except Exception as e:
            self._logger.error(e)

    def submit_task(self, new_task: Task) -> Task:
        """
        Puts a task returned by asyncio.ensure_future to the task_cache to prevent garbage collection and allow return
        value analysis
        :param new_task: An asyncio.Task
        :return: The submitted task, if plugin specific callbacks are added
        """
        new_task.add_done_callback(partial(self._task_callback))
        self._task_cache.append(new_task)
        return new_task
