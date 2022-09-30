import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, server_utility

from .abstract_base_plugin import AbstractBasePlugin


class ServerUtility(AbstractBasePlugin):
    """
    Utility for onboard MAVSDK instances for common “server” tasks.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("server_utility", system, loop, logger)

    def send_status_text(self, typ: server_utility.StatusTextType, text):
        """
        Sends a status and a description of the status
        :param typ: a mavsdk.server_utility.StatusTextType item describing the status of the system
        :param text: a string object as a description or note
        :return: None
        """
        self._logger.info(
            'Sent a "{typ}" status to the server with the message "{text}"'
        )
        super().submit_coroutine(
            self._system.server_utility.send_status_text(typ, text)
        )
