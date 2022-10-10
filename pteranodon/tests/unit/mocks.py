from logging import Logger
import asyncio
import threading

from mavsdk import System
import pytest


@pytest.fixture
def mock_logger() -> Logger:
    """
    A Logger object for each test
    """
    return Logger("mock")


@pytest.fixture(scope="session")
def mock_loop():
    """
    A Logger object for each test
    """
    loop = asyncio.get_event_loop()
    t = threading.Thread(target=loop.run_forever, daemon=True)
    t.start()

    yield loop
    loop.stop()
    t.join(timeout=1.0)


@pytest.fixture
def mock_system():
    """
    A System object to be setup and torn down for each test
    """
    # Setup
    sys = System()

    yield sys

    # Teardown
    sys.__del__()  # pylint: disable=unnecessary-dunder-call
    del sys
