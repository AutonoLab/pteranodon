import asyncio
from logging import Logger
import concurrent.futures as c_futures

from mavsdk import System
import pytest

from ...plugins.abstract_plugin import AbstractPlugin
from .mocks import (  # noqa: F401 # pylint: disable=unused-import # (Needed for fixtures)
    mock_system,
    mock_logger,
    mock_loop,
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
    mock_loop: asyncio.AbstractEventLoop,  # noqa: F811 # pylint: disable=redefined-outer-name # (Needed for fixtures)
):
    """
    Tests the AbstractPlugin submit_task method using the counting_function.

    :param mock_system: The system fixture, imported from mocks
    :type mock_system: System
    :param mock_logger: The logger fixture, imported from mocks
    :type mock_logger: Logger
    """

    mock_plugin = AbstractPlugin("mock", mock_system, mock_loop, mock_logger)

    sum_of_five = 5 + 4 + 3 + 2 + 1 + 0

    counting_future = mock_plugin._submit_coroutine(  # pylint: disable=protected-access
        counting_function(5)
    )

    try:
        assert (
            counting_future.result(1.0) == sum_of_five
        ), "Result is not equal to the expected value"
        assert any(
            sum_of_five in x
            for x in mock_plugin._result_cache  # pylint: disable=protected-access
        ), "Result is not in the result cache"  # Don't know name so must test with any

    except c_futures.TimeoutError:
        pytest.fail("Single task timed out!", pytrace=True)
    except Exception as e:
        pytest.fail(f"Single task raised an exception: {e}")
    else:
        assert True
