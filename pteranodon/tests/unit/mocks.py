import asyncio

from mavsdk import System
from logging import Logger
import pytest


@pytest.fixture
def mock_logger() -> Logger:
    return Logger("mock")


@pytest.fixture
def mock_system() -> System:
    sys = System()
    return sys
