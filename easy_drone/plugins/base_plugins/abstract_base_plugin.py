from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System

from ..abstract_plugin import AbstractPlugin


class AbstractBasePlugin(AbstractPlugin):
    def __init__(self, name: str, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__(name, system, loop, logger)
        