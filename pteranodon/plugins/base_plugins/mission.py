import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from mavsdk import System, mission
from .abstract_base_plugin import AbstractBasePlugin


class Mission(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission", system, loop, logger)
        self._download_progress = None
        self._enable_RTL = None
        self._mission_progress = None

    def cancel_mission_download(self):
        self._logger.info("Canceled Mission Download")
        super().submit_task(
            asyncio.ensure_future(self._system.mission.cancel_mission_download(), loop=self._loop)
        )

    def cancel_mission_upload(self):
        self._logger.info("Canceled Mission Upload")
        super().submit_task(
            asyncio.ensure_future(self._system.mission.cancel_mission_upload(), loop=self._loop)
        )

    def clear_mission(self):
        self._logger.info("Cleared mission")
        super().submit_task(
            asyncio.ensure_future(self._system.mission.clear_mission(), loop=self._loop)
        )

    def download_mission(self) -> mission.MissionPlan:
        self._logger.info("Downloading mission file")
        return_mission = self._system.mission.download_mission()
        self._logger.info("Mission file download complete")
        return return_mission

    def download_mission_with_progress(self):
        async for progress in self._system.mission.download_mission_with_progress():
            if type(progress) != mission.MissionPlan:
                self._download_progress = progress
            break

    def get_return_to_launch_after_mission(self) -> bool:
        return self._system.mission.get_return_to_launch_after_mission()

    def is_mission_finished(self) -> bool:
        return self._system.mission.is_mission_finished()

    def mission_progress(self):
        async for progress in self._system.mission.mission_progress():
            self._mission_progress = progress

    