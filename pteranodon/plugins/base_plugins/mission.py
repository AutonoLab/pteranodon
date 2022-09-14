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
        """
        Cancels the current mission download
        :return: None
        """
        self._logger.info("Canceled Mission Download")
        super().submit_task(
            asyncio.ensure_future(self._system.mission.cancel_mission_download(), loop=self._loop)
        )

    def cancel_mission_upload(self):
        """
        cancels the current mission upload
        :return: None
        """
        self._logger.info("Canceled Mission Upload")
        super().submit_task(
            asyncio.ensure_future(self._system.mission.cancel_mission_upload(), loop=self._loop)
        )

    def clear_mission(self):
        """
        Clears the current mission
        :return: None
        """
        self._logger.info("Cleared mission")
        super().submit_task(
            asyncio.ensure_future(self._system.mission.clear_mission(), loop=self._loop)
        )

    # OPTIONAL METHOD DEFINITION TO ADD A TIMEOUT PERIOD WITH A 1 SECOND DEFAULT VALUE
    # def download_mission(self, timeout_period: float = 1) -> mission.MissionPlan:
    def download_mission(self) -> mission.MissionPlan:
        """
        Returns the current mission plan
        :return: mission.MissionPlan
        """
        self._logger.info("Downloading mission file")

        download_mission_task = asyncio.ensure_future(self._system.mission.download_mission(), loop=self._loop)
        done_condition = Condition()
        download_mission_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        # OPTIONAL TO ADD A TIMEOUT PARAM TO REDUCE TIMEOUT ERRORS WHILE INCREASING BLOCKED THREAD TIME
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

    async def _download_mission_with_progress(self) -> mission.MissionPlan:
        """
        updates mission progress, and when complete returns the current mission plan
        :return: mission.MissionPlan
        """
        async for progress in self._system.mission.download_mission_with_progress():
            if progress.has_mission:
                self._mission_plan = progress.mission_plan
                return progress.mission_plan
            elif progress.has_progress:
                self._logger.info(f"Mission Download at {progress.progress * 100}%")
                self._mission_progress = progress

    def download_mission_with_progress(self) -> mission.MissionPlan:
        """
        Starts a download of the Mission plan which updates self._mission_progress with the downloads
        progress
        :return: mission.MissionPlan
        """
        self._logger.info("Downloading mission file with progress information")

        # Block a thread and allow it to run for 1 second before timing out
        download_mission_task = asyncio.ensure_future(self._download_mission_with_progress(), loop=self._loop)
        done_condition = Condition()
        download_mission_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        # Test if any information was returned or the function timed out
        try:
            x = download_mission_task.result()
            self._logger.info("Mission file downloaded successfully")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Could not download mission file! Request timed out!")
            return []

    def get_return_to_launch_after_mission(self) -> bool:
        """
        retrieves the boolean that determines if it returns to the launch location or stays at current location
        :return: boolean
        """
        self._logger.info("Waiting for response to get_return_to_launch_after_mission()")

        # Block a thread and allow it to run for 1 second before timing out
        get_rtl_task = asyncio.ensure_future(self._system.mission.get_return_to_launch_after_mission(), loop=self._loop)
        done_condition = Condition()
        get_rtl_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        # Test if any information was returned or the function timed out
        try:
            x = get_rtl_task.result()
            self._logger.info("Response to get_return_to_launch_after_mission() received")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Could not retrieve RTL information! Request timed out!")
            return []

    def is_mission_finished(self) -> bool:
        """
        retrieves the boolean that states the current status of the mission
        :return: boolean
        """
        self._logger.info("Waiting for response to is_mission_finished()")

        # Block a thread and allow it to run for 1 second before timing out
        get_imf_task = asyncio.ensure_future(self._system.mission.is_mission_finished(), loop=self._loop)
        done_condition = Condition()
        get_imf_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        # Test if any information was returned or the function timed out
        try:
            x = get_imf_task.result()
            self._logger.info("Response to is_mission_finished received")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("is_mission_finished request timed out!")
            return []

    def mission_progress(self) -> mission.MissionProgress:
        """
        returns the current mission progress
        :return: mission.MissionProgress
        """
        return self._mission_progress

    async def _update_mission_progress(self):
        """
        updates the mission progress
        :return: None
        """
        async for progress in self._system.mission.mission_progress():
            self._mission_progress = progress
