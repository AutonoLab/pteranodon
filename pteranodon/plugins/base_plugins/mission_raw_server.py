from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, mission_raw_server

from .abstract_base_plugin import AbstractBasePlugin


class MissionRawServer(AbstractBasePlugin):
    """
    Acts as a vehicle and receives incoming missions from GCS (in raw MAVLINK format). Provides current mission item
    state, so the server can progress through missions.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission_raw_server", system, loop, logger)

        self._submit_simple_generator(self._system.mission_raw_server.clear_all)
        self._submit_simple_generator(
            self._system.mission_raw_server.current_item_changed
        )
        self._submit_simple_generator(self._system.mission_raw_server.incoming_mission)

        self._end_init()

    @property
    def mission_plan(self) -> Optional[mission_raw_server.MissionPlan]:
        """
        Returns current mission plan
        """
        return self._async_gen_data[self._system.mission_raw_server.incoming_mission]

    @property
    def clear_type(self) -> Optional[int]:
        """
        Returns last clear_type received
        """
        return self._async_gen_data[self._system.mission_raw_server.clear_all]

    @property
    def mission_item(self) -> Optional[mission_raw_server.MissionItem]:
        """
        Returns current mission items
        """
        return self._async_gen_data[
            self._system.mission_raw_server.current_item_changed()
        ]

    def set_current_item_complete(self):
        """
        sets the current mission item to complete
        :return:
        """
        self._logger.info("Task item set to complete")
        self._submit_coroutine(
            self._system.mission_raw_server.set_current_item_complete()
        )
