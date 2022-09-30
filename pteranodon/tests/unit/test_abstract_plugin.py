import asyncio
from threading import Condition
from logging import Logger

from mavsdk import System
import pytest

from ...plugins.abstract_plugin import AbstractPlugin
from .mocks import (  # noqa: F401 # pylint: disable=unused-import # (Needed for fixtures)
    mock_system,
    mock_logger,
)


async def counting_function(num: int) -> int:
    """
    A testing async function that simply sums from 0..num (:math:`\\sum_{i=0}^{num}i`)

    :param num: value to sum to
    :type num: integer
    :return: sum
    :rtype: integer
    """
    value_sum = 0
    for i in range(num + 1):
        value_sum += i
    return value_sum


def test_submit_task(
    mock_system: System,  # noqa: F811 # pylint: disable=redefined-outer-name # (Needed for fixtures)
    mock_logger: Logger,  # noqa: F811 # pylint: disable=redefined-outer-name # (Needed for fixtures)
):
    """
    Tests the AbstractPlugin submit_task method using the counting_function.

    :param mock_system: The system fixture, imported from mocks
    :type mock_system: System
    :param mock_logger: The logger fixture, imported from mocks
    :type mock_logger: Logger
    """

    loop = asyncio.get_event_loop()

    mock_plugin = AbstractPlugin("mock", mock_system, loop, mock_logger)

    condition = Condition()

    sum_of_five = 5 + 4 + 3 + 2 + 1 + 0

    counting_task = asyncio.run_coroutine_threadsafe(counting_function(5), loop=loop)
    counting_task.add_done_callback(lambda _: condition.notify())

    mock_plugin.submit_task(counting_task)

    loop.run_until_complete(counting_task)

    with condition:
        condition.wait(1.0)

    try:
        assert (
            counting_task.result() == sum_of_five
        ), "Result is not equal to the expected value"
        assert (
            counting_task in mock_plugin._task_cache  # pylint: disable=protected-access
        ), "Task is not in the task cache"
        assert (
            sum_of_five in mock_plugin._result_cache  # pylint: disable=protected-access
        ), "Result is not in the result cache"
    except asyncio.InvalidStateError:
        # If the result is not available yet,
        #       it can be assumed that the wait call timed out before the callback was done
        pytest.fail("Single task timed out!", pytrace=True)
