from asyncio import Task

import pytest

from ..helpers import condition_notify_result
from pteranodon.simple_drone import SimpleDrone
from pteranodon.plugins.base_plugins import Offboard

# To run docker container:
# docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.0

# To run tests
# pytest --log-cli-level=INFO --full-trace -rP tests/integration/test_offboard.py


@pytest.fixture
def test_drone() -> SimpleDrone:
    # Setup
    print("Setup")
    test_drone = SimpleDrone("udp://:14540")

    yield test_drone

    # Teardown
    print("Teardown")
    test_drone.stop()


def test_start(test_drone):
    offboard_plugin: Offboard = test_drone.offboard

    test_drone.arm()

    # Running on offboard plugin so that the task cache is updated in real time.
    offboard_plugin.start()

    start_task: Task = offboard_plugin._task_cache[-1]

    result = pytest.helpers.condition_notify_result(start_task, 1.0)

    assert result is None


@pytest.mark.depends(on=["test_start"])
def test_hold(test_drone):
    offboard_plugin: Offboard = test_drone.offboard

    test_drone.arm()

    # Running on offboard plugin so that the task cache is updated in real time.
    offboard_plugin.start()

    offboard_plugin.hold()

    hold_task: Task = offboard_plugin._task_cache[-1]

    result = pytest.helpers.condition_notify_result(hold_task, 1.0)

    assert result is None
