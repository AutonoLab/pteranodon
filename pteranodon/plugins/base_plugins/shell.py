import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import List

from mavsdk import System, info

from .abstract_base_plugin import AbstractBasePlugin



class Shell(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("shell", system, loop, logger)

        self._feedback_history : List[str] = []

    async def _receive_data(self) -> None:
        async for data in self._system.shell.receive():
            self._feedback_history.append(data)

    def get_newest_feedback(self) -> str:
        return self._feedback_history[-1]

    @property
    def feedback_history(self) -> List[str]:
        return self._feedback_history


