import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from time import sleep
from typing import List, Dict, Any

from mavsdk import System, failure
from mavsdk.failure import FailureUnit, FailureType

from .abstract_base_plugin import AbstractBasePlugin

class Failure(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("failure", system, loop, logger)

    def inject(self, failure_unit : FailureUnit, failure_type : FailureType, instance : int) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.failure.inject(failure_unit, failure_type, instance))
        )


