import pytest
import asyncio
import typing
from threading import Condition
from asyncio import Task


@pytest.helpers.register
def condition_notify_result(task : Task, timeout : float) -> typing.Optional[typing.Any]:
    condition = Condition()

    def func():
        with condition:
            condition.notify()

    task.add_done_callback(lambda _: func())

    with condition:
        condition.wait(timeout)

    try:
        exception = task.exception()
        if exception is not None:
            raise exception

    except asyncio.InvalidStateError:
        pass

    try:
        return task.result()
    except asyncio.InvalidStateError:
        pass

    return None