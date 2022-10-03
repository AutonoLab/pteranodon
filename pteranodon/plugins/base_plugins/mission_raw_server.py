from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional
from functools import partial

from mavsdk import System, mission_raw_server

from .abstract_base_plugin import AbstractBasePlugin


class MissionRawServer(AbstractBasePlugin):
    """
    Acts as a vehicle and receives incoming missions from GCS (in raw MAVLINK format). Provides current mission item
    state, so the server can progress through missions.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission_raw_server", system, loop, logger)

        self._mission_plan: Optional[mission_raw_server.MissionPlan] = None
        self._clear_type: Optional[int] = None
        self._mission_item: Optional[mission_raw_server.MissionItem] = None

        self._submit_generator(partial(self._clear_all))
        self._submit_generator(partial(self._current_item_changed))
        self._submit_generator(partial(self._incoming_mission))

        self._end_init()

    @property
    def mission_plan(self) -> Optional[mission_raw_server.MissionPlane]:
        """
        Returns current mission plan
        """
        return self._mission_plan

    @property
    def clear_type(self) -> Optional[int]:
        """
        Returns last clear_type received
        """
        return self._clear_type

    @property
    def mission_item(self) -> Optional[mission_raw_server.MissionItem]:
        """
        Returns current mission items
        """
        return self._mission_item

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
        self._submit_coroutine(
            self._system.mission_raw_server.set_current_item_complete()
        )
