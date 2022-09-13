import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from time import sleep
from typing import List, Dict, Any

from mavsdk import System, failure
#from mavsdk.failure import... not sure if I need something here

from .abstract_base_plugin import AbstractBasePlugin

class Failure(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("failure", system, loop, logger)



