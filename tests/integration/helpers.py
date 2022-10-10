import asyncio
import functools
import typing
from typing import Optional, Any
from threading import Condition
from asyncio import Task, Future
from pteranodon.plugins import AbstractPlugin


class PluginTaskFetcher:
    def __init__(self, plugin: AbstractPlugin):

        self._condition = Condition()
        self._fetched_task: Optional[Task] = None
        self._fetching_plugin: AbstractPlugin = plugin
        self._fetched_task_num: int = 1

    def _notify_condition(self, *args):
        with self._condition:
            self._condition.notify()

    def trigger(self):
        """
        Method which is called when the task is now available. Typically passed in the `put` function.
        """
        self._fetched_task = self._fetching_plugin._future_cache[-1]

        self._notify_condition()

    def fetch(self, timeout: float) -> Optional[Task]:
        """
        Waits until the `trigger` method has been called or the specified timeout has occurred to return the fetched task.
        :param timeout: The amount of time (in seconds) to wait for the task to become available
        :type timeout: float
        :return: The fetched task, None if the operation timed out
        :rtype: Optional[Task]
        """
        with self._condition:
            self._condition.wait(timeout)

        return self._fetched_task

    def __callbacks_contains_func(self, func_name: str) -> bool:
        for callback in self._fetched_task._done_callbacks:
            func: typing.Callable = callback
            if isinstance(func, functools.partial):
                func = callback.func

            if func.__qualname__.endswith(func_name):
                return True

        return False

    def _scheduled_notify_callback(self, finished_future):

        # Assumes the ordering specified in _submit_coroutine where the _future_callback is called before
        #   the schedule_callback and this callback is added after both of them

        future_cache = self._fetching_plugin._future_cache

        # If cache is empty, no more scheduled coroutines (can't preemptively check if > 1, I don't know why)
        if len(future_cache) <= 0:
            self._notify_condition()
            return

        # Update to the newest task
        self._fetched_task = future_cache[-1]

        # If this task is also in the schedule chain, recurse
        if self.__callbacks_contains_func("_schedule"):
            print(
                f"Previously fetched task has another task {self._fetched_task} in the schedule chain, recursing"
            )
            self._fetched_task.add_done_callback(self._scheduled_notify_callback)
            return

        # Otherwise, notify the condition that we have reached the final condition
        self._fetched_task.add_done_callback(self._notify_condition)

    def fetch_result(self, timeout: float) -> Optional[Any]:
        """
        After the task has been fetched, waits for the task to finish execution and returns the result
        :param timeout: The amount of time (in seconds) to wait for the task to complete
        :type timeout: float
        :return: The return value which can be None or Any value
        :rtype: Optional[Any]
        :raises: If no Task has been fetched, RuntimeError, otherwise the error that the task raises.
        """
        if self._fetched_task is None:
            raise RuntimeError("Task not fetched yet!")

        # If this task is only one part of a scheduled task, wait until all finish
        if self.__callbacks_contains_func("_schedule"):
            self._fetched_task.add_done_callback(self._scheduled_notify_callback)

        with self._condition:
            self._condition.wait(timeout)

        try:
            exception = self._fetched_task.exception()
            if exception is not None:
                raise exception
        # Either the task timed out or there is no exception set
        except asyncio.InvalidStateError:
            pass

        try:
            return self._fetched_task.result()
        # Either the task timed out or there is no result set
        except asyncio.InvalidStateError:
            pass

        return None
