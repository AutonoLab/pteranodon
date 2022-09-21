import asyncio

from mavsdk import System
from logging import Logger
import pytest


@pytest.fixture
def mock_logger() -> Logger:
    return Logger("mock")


@pytest.fixture
def mock_system():
    # Setup
    sys = System()

    yield sys

    # Teardown
    sys.__del__()
    del sys
