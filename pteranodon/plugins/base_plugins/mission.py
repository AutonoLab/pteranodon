from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, mission

from .abstract_base_plugin import AbstractBasePlugin


class Mission(AbstractBasePlugin):
    """
    Enable waypoint missions.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission", system, loop, logger)

        self._download_progress = None
        self._enable_return_to_land = None
        self._mission_plan = None
        self._loop.run_until_complete(self._download_mission_with_progress())
        self._submit_simple_generator(self._system.mission.mission_progress())

        self._end_init()

    def cancel_mission_download(self):
        """
        Cancels the current mission download
        :return: None
        """
        self._logger.info("Canceled Mission Download")
        self._submit_coroutine(self._system.mission.cancel_mission_download())

    def cancel_mission_upload(self):
        """
        cancels the current mission upload
        :return: None
        """
        self._logger.info("Canceled Mission Upload")
        self._submit_coroutine(self._system.mission.cancel_mission_upload())

    def clear_mission(self):
        """
        Clears the current mission
        :return: None
        """
        self._logger.info("Cleared mission")
        self._submit_coroutine(self._system.mission.clear_mission())

    def download_mission(self, timeout: float = 1.0) -> Optional[mission.MissionPlan]:
        """
        Returns the current mission plan
        :return: mission.MissionPlan
        """
        self._logger.info("Downloading mission file")

        downloaded_mission = self._submit_blocking_coroutine(
            self._system.mission.download_mission(), timeout=timeout
        )

        if downloaded_mission is not None:
            self._logger.info("Mission file downloaded successfully")
        else:
            self._logger.error("Could not download mission file! Request timed out!")
        return downloaded_mission

    async def _download_mission_with_progress(self) -> None:
        """
        updates mission progress, and when complete returns the current mission plan
        :return: mission.MissionPlan
        """
        async for progress in self._system.mission.download_mission_with_progress():
            if progress.has_mission:
                self._mission_plan = progress.mission_plan
                return
            if progress.has_progress:
                self._logger.info(f"Mission Download at {progress.progress * 100}%")
                self._mission_progress = progress

    def download_mission_with_progress(
        self, timeout: float = 1.0
    ) -> Optional[mission.MissionPlan]:
        """
        Starts a download of the Mission plan which updates self._mission_progress with the downloads
        progress
        :return: mission.MissionPlan
        """
        self._logger.info("Downloading mission file with progress information")

        mission_download_progess = self._submit_blocking_coroutine(
            self._download_mission_with_progress(), timeout=timeout
        )

        if mission_download_progess is not None:
            self._logger.info("Mission file downloaded successfully")
        else:
            self._logger.error("Could not download mission file! Request timed out!")
        return mission_download_progess

    def get_return_to_launch_after_mission(
        self, timeout: float = 1.0
    ) -> Optional[bool]:
        """
        retrieves the boolean that determines if it returns to the launch location or stays at current location
        :return: boolean
        """
        self._logger.info(
            "Waiting for response to get_return_to_launch_after_mission()"
        )

        rtl_state = self._submit_blocking_coroutine(
            self._system.mission.get_return_to_launch_after_mission(),
            timeout=timeout,
        )

        if rtl_state is not None:
            self._logger.info(
                "Response to get_return_to_launch_after_mission() received"
            )
        else:
            self._logger.error("Could not retrieve RTL information! Request timed out!")
        return rtl_state

    def is_mission_finished(self, timeout: float = 1.0) -> Optional[bool]:
        """
        retrieves the boolean that states the current status of the mission
        :return: boolean
        """
        self._logger.info("Waiting for response to is_mission_finished()")

        imf_state = self._submit_blocking_coroutine(
            self._system.mission.is_mission_finished(), timeout=timeout
        )

        if imf_state is not None:
            self._logger.info("Response to is_mission_finished received")
        else:
            self._logger.error("is_mission_finished request timed out!")
        return imf_state

    def mission_progress(self) -> Optional[mission.MissionProgress]:
        """
        returns the current mission progress
        :return: mission.MissionProgress
        """
        return self._async_gen_data[self._system.mission.mission_progress()]
