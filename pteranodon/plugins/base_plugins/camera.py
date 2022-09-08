import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List

from mavsdk import System

from .abstract_base_plugin import AbstractBasePlugin



class Camera(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("camera", system, loop, logger)