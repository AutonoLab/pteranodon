from asyncio import Task

import pytest

from ...simple_drone import SimpleDrone
from .helpers import condition_notify_result
from ...plugins.base_plugins import Offboard

# To run docker container:
# docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.0


@pytest.fixture(scope='session')
def test_drone() -> SimpleDrone:
    # Setup
    test_drone = SimpleDrone("udp://:14540")


    yield test_drone

    # Teardown
    test_drone.stop()


def test_hold(test_drone):
    offboard_plugin: Offboard = test_drone.offboard

    initial_result_cache_len = len(offboard_plugin._result_cache)
    print(initial_result_cache_len)


    test_drone.arm()

    # Running on offboard plugin so that the task cache is updated in real time.
    offboard_plugin.start()

    offboard_plugin.hold()

    hold_task: Task = offboard_plugin._task_cache[-1]


    result = pytest.helpers.condition_notify_result(hold_task, 1.0)


    assert result is None




