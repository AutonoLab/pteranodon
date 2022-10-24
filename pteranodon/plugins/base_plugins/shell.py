from asyncio import AbstractEventLoop
from logging import Logger
from typing import List, Callable

from mavsdk import System

from .abstract_base_plugin import AbstractBasePlugin


class Shell(AbstractBasePlugin):
    """
    Allow to communicate with the vehicle’s system shell.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("shell", system, loop, logger)

        self._feedback_history: List[str] = []
        self._cmd_history: List[str] = []

        self._submit_simple_generator(self._system.shell.receive)
        self._register_handler(self._system.shell.receive)(self._update_feedback)

        self._end_init()

    def _update_feedback(self, data):
        self._feedback_history.append(data)

    def send(self, command: str) -> None:
        """
        Send a command. This command will be added to the cmd_history.

        :param command: The command to send
        """
        self._submit_coroutine(self._system.shell.send(command))
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

    def register_receive_handler(self, handler: Callable) -> None:
        """
        Registers a function (Callable) to be a handler of the data stream
        :param handler: A Callable which gets executed each time new data is received
        """
        self._register_handler(self._system.shell.receive)(handler)
