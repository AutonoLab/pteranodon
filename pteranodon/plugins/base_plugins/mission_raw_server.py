import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, mission_raw_server

from .abstract_base_plugin import AbstractBasePlugin


class MissionRawServer(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission_raw_server", system, loop, logger)
        self.mission_plan: Optional[mission_raw_server.MissionPlan] = None
        self.clear_type: Optional[int] = None
        self.mission_item: Optional[mission_raw_server.MissionItem] = None
        self._clear_all_task = asyncio.ensure_future(self._clear_all(), loop=self._loop)
        self._current_item_changed_task = asyncio.ensure_future(self._current_item_changed(), loop=self._loop)
        self._incoming_mission_task = asyncio.ensure_future(self._incoming_mission, loop=self._loop)

    async def _clear_all(self):
        """
        updates the clear_type
        :return: None
        """
        async for x in self._system.mission_raw_server.clear_all():
            if x != self.clear_type:
                self.clear_type = x

    async def _current_item_changed(self):
        """
        updates the current mission item
        :return: None
        """
        async for x in self._system.mission_raw_server.current_item_changed():
            if x != self.mission_item:
                self.mission_item = x

    async def _incoming_mission(self):
        """
        sets new mission_item
        :return: None
        """
        async for x in self._system.mission_raw_server.incoming_mission():
            if x != self.mission_plan:
                self.mission_plan = x

    def set_current_item_complete(self):
        """
        sets the current mission item to complete
        :return:
        """
        self._logger.info("Task item set to complete")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw_server.set_current_item_complete(), loop=self._loop)
        )
