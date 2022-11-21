from typing import Optional
from asyncio import Task

import pytest

from .helpers import PluginTaskFetcher
from pteranodon.simple_drone import SimpleDrone
from pteranodon.utils import ServerDetector
from pteranodon.plugins.base_plugins import Offboard
from threading import Condition

# To run docker container:
# docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.0

# To run testsg
# pytest --log-cli-level=INFO --full-trace -rP tests/integration/test_offboard.py


@pytest.fixture(scope="session")
def test_drone(hostname, port) -> SimpleDrone:
    # Setup
    print("Setup")
    print(f"Host: {hostname}, Port: {port}")
    test_drone = SimpleDrone(f"udp://{hostname}:{port}")

    yield test_drone

    # Teardown
    print("Teardown")
    test_drone.stop()


def test_start(test_drone):
    offboard_plugin: Offboard = test_drone.offboard

    test_drone.put(test_drone.action.arm)

    task_fetcher = PluginTaskFetcher(offboard_plugin)

    start_cmd = SimpleDrone.Command(offboard_plugin.start, handler=task_fetcher.trigger)

    test_drone.put(start_cmd)

    start_task = task_fetcher.fetch(5.0)

    assert start_task is not None, "Start task was not fetched!"

    result = task_fetcher.fetch_result(20.0)

    assert result is None, "Start Task returned an exception (not None)"


@pytest.mark.depends(on=["test_start"])
def test_hold(test_drone):
    offboard_plugin: Offboard = test_drone.offboard

    test_drone.put(test_drone.action.arm)

    # Running on offboard plugin so that the task cache is updated in real time
    test_drone.put(offboard_plugin.start)

    task_fetcher = PluginTaskFetcher(offboard_plugin)

    hold_cmd = SimpleDrone.Command(offboard_plugin.hold, handler=task_fetcher.trigger)

    test_drone.put(hold_cmd)

    hold_task = task_fetcher.fetch(5.0)

    assert hold_task is not None, "Hold task was not fetched!"

    result = task_fetcher.fetch_result(20.0)

    assert result is None, "Hold Task returned an exception (not None)"
