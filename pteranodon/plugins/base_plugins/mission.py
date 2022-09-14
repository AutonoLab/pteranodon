import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, mission
from threading import Condition

from .abstract_base_plugin import AbstractBasePlugin


class Mission(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission", system, loop, logger)
        self._download_progress = None
        self._enable_RTL = None
        self._mission_plan = None
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

    # OPTIONAL METHOD DEFINITION TO ADD A TIMEOUT PERIOD WITH A 1 SECOND DEFAULT VALUE
    # def download_mission(self, timeout_period: float = 1) -> mission.MissionPlan:
    def download_mission(self) -> mission.MissionPlan:
        self._logger.info("Downloading mission file")

        download_mission_task = asyncio.ensure_future(self._system.mission.download_mission(), loop=self._loop)
        done_condition = Condition()

        # When task is done, stop waiting
        download_mission_task.add_done_callback(lambda _: done_condition.notify())

        # Wait with a timeout of 1 second
        done_condition.wait(1.0)

        # OPTIONAL TO ADD A TIMEOUT PARAM TO REDUCE TIMEOUT ERRORS, OR DECREASE CPU IDLE TIME
        # done_condition.wait(timeout_period)

        try:
            x = download_mission_task.result()
            self._logger.info("Mission file downloaded successfully")
            return x
        except asyncio.InvalidStateError:
            # If the result is not available yet,
            #       it can be assumed that the wait call timed out before the callback was done
            self._logger.error("Could not download mission file! Request timed out!")
            return []

    async def _download_mission_with_progress(self):
        async for progress in self._system.mission.download_mission_with_progress():
            if progress.has_mission:
                self._mission_plan = progress.mission_plan
            elif progress.has_progress:
                self._logger.info(f"Mission Download at {progress * 100}%")
                self._mission_progress = progress

    # OPTIONAL METHOD DEFINITION TO ADD A TIMEOUT PERIOD WITH A 1 SECOND DEFAULT VALUE
    # def download_mission_with_progress(self, timeout_period: float = 1.0)
    def download_mission_with_progress(self):
        self._logger.info("Downloading mission file with progress information")

        download_mission_task = asyncio.ensure_future(self._download_mission_with_progress(), loop=self._loop)
        done_condition = Condition()

        # When task is done, stop waiting
        download_mission_task.add_done_callback(lambda _: done_condition.notify())

        # Wait with a timeout of 1 second
        done_condition.wait(1.0)

        # OPTIONAL TO ADD A TIMEOUT PARAM TO REDUCE TIMEOUT ERRORS, OR DECREASE CPU IDLE TIME
        # done_condition.wait(timeout_period)

        try:
            x = download_mission_task.result()
            self._logger.info("Mission file downloaded successfully")
            return x
        except asyncio.InvalidStateError:
            # If the result is not available yet,
            #       it can be assumed that the wait call timed out before the callback was done
            self._logger.error("Could not download mission file! Request timed out!")
            return []

    def get_return_to_launch_after_mission(self):
        return self._system.mission.get_return_to_launch_after_mission()

    def is_mission_finished(self) -> bool:
        return self._system.mission.is_mission_finished()

    def mission_progress(self):
        async for progress in self._system.mission.mission_progress():
            self._mission_progress = progress
