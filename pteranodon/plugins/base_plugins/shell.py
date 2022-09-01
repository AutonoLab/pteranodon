import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger

from mavsdk import System, info

from .abstract_base_plugin import AbstractBasePlugin


class Shell(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("shell", system, loop, logger)
