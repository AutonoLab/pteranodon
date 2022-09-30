import pytest
import asyncio
import typing
from typing import Optional, Any
from threading import Condition
from asyncio import Task, Future
from pteranodon.plugins import AbstractPlugin
from grpc._channel import _MultiThreadedRendezvous


class PluginTaskFetcher:

    def __init__(self, plugin : AbstractPlugin):

        self._condition = Condition()
        self._fetched_task: Optional[Task] = None
        self._fetching_plugin : AbstractPlugin = plugin

    def _notify_condition(self):
        with self._condition:
            self._condition.notify()

    def trigger(self):
        """
        Method which is called when the task is now available. Typically passed in the `put` function.
        """
        self._fetched_task = self._fetching_plugin._task_cache[-1]

        self._notify_condition()

    def fetch(self, timeout : float) -> Optional[Task]:
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

    def fetch_result(self, timeout : float) -> Optional[Any]:
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

        self._fetched_task.add_done_callback(self._notify_condition)

        with self._condition:
            self._condition.wait(timeout)

        try:
            exception = self._fetched_task.exception()
            if exception is not None:
                if isinstance(exception, _MultiThreadedRendezvous):
                    # Just for testing
                    return None
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
