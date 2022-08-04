from abc import ABC
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System


class AbstractPlugin(ABC):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        self._system = system
        self._loop = loop
        self._logger = logger
