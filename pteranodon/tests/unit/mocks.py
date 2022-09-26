from logging import Logger

from mavsdk import System
import pytest


@pytest.fixture
def mock_logger() -> Logger:
    """
    A Logger object for each test
    """
    return Logger("mock")


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
