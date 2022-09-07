import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List

from mavsdk import System, ftp

from .abstract_base_plugin import AbstractBasePlugin



class Ftp(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("ftp", system, loop, logger)
        self._comp_id = None

        self._comp_id_task = asyncio.ensure_future(self._get_our_compid(), loop=self._loop)

    async def _get_our_compid(self) -> None:
        while True:
            self._id = await self._system.ftp.get_our_compid()
            break

    def get_our_component_id(self) -> int:
        """
        Get our own component ID.

        :return: Our component ID
        :rtype: uint32
        """
        return self._comp_id
