from ...plugins.abstract_plugin import AbstractPlugin
from mavsdk import System
from logging import Logger
import pytest
import asyncio
from threading import Condition


@pytest.fixture
def mock_logger() -> Logger:
    return Logger("mock")


@pytest.fixture
def mock_system() -> System:
    return System()


async def counting_function(num : int) -> int:
    value_sum = 0
    for i in range(num+1):
        value_sum += i
    return value_sum


def test_submit_task(mock_system : System, mock_logger : Logger):
    loop = asyncio.get_event_loop()

    mock_plugin = AbstractPlugin("mock", mock_system, loop, mock_logger)

    condition = Condition()

    sum_of_five = (5 + 4 + 3 + 2 + 1)

    counting_task = asyncio.ensure_future(counting_function(5), loop=loop)
    counting_task.add_done_callback(lambda _: condition.notify())

    mock_plugin.submit_task(counting_task)

    loop.run_until_complete(counting_task)

    with condition:
        condition.wait(1.0)

    try:
        assert (counting_task.result() == sum_of_five), "Result is not equal to the expected value"
        assert (counting_task in mock_plugin._task_cache), "Task is not in the task cache"
        assert (sum_of_five in mock_plugin._result_cache), "Result is not in the result cache"
    except asyncio.InvalidStateError:
        # If the result is not available yet,
        #       it can be assumed that the wait call timed out before the callback was done
        pytest.fail("Single task timed out!", pytrace=True)


