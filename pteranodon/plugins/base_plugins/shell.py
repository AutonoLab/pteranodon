import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List

from mavsdk import System

from .abstract_base_plugin import AbstractBasePlugin



class Shell(AbstractBasePlugin):
    """
    Allow to communicate with the vehicle’s system shell.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("shell", system, loop, logger)

        self._receive_task = asyncio.ensure_future(self._receive_feedback(), loop=self._loop)
        self._feedback_history: List[str] = []
        self._cmd_history: List[str] = []

    async def _receive_feedback(self) -> None:
        """
        Receive feedback from a sent command.
        """
        async for data in self._system.shell.receive():
            self._feedback_history.append(data)

    def send(self, command: str) -> None:
        """
        Send a command. This command will be added to the cmd_history.

        :param command: The command to send
        """
        super().submit_task(
            asyncio.ensure_future(self._system.shell.send(command), loop=self._loop)
        )
        self._cmd_history.append(command)

    def get_newest_feedback(self) -> str:
        """
        Return's the most recent feedback received from the MAV SDK

        :return: feedback data string
        """
        return self._feedback_history[-1]

    def get_newest_cmd(self) -> str:
        """
        Return's the most recent command sent to MAV SDK

        :return: command string
        """
        return self._feedback_history[-1]

    @property
    def feedback_history(self) -> List[str]:
        """
        The history of the feedback received, with the first element being the oldest

        :return: a list of feedback strings
        """
        return self._feedback_history

    @property
    def cmd_history(self) -> List[str]:
        """
        The history of the commands sent, with the first element being the oldest

        :return: a list of command strings
        """
        return self._cmd_history


