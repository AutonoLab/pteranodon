import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, mission_raw
from threading import Condition

from .abstract_base_plugin import AbstractBasePlugin


class MissionRaw(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission_raw", system, loop, logger)
        self._mission_progress = None
        self.has_mission_changed = False
        self._mission_changed_task = asyncio.ensure_future(self._update_mission_changed(), loop=self._loop)
        self._mission_progress_task = asyncio.ensure_future(self._update_mission_progress(), loop=self._loop)

    def cancel_mission_download(self):
        """
        Cancels the current mission download
        :return: None
        """
        self._logger.info("Canceled Mission Download")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw.cancel_mission_download(), loop=self._loop)
        )

    def cancel_mission_upload(self):
        """
        cancels the current mission upload
        :return: None
        """
        self._logger.info("Canceled Mission Upload")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw.cancel_mission_upload(), loop=self._loop)
        )

    def clear_mission(self):
        """
        Clears the current mission
        :return: None
        """
        self._logger.info("Cleared mission")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw.clear_mission(), loop=self._loop)
        )

    # OPTIONAL METHOD DEFINITION TO ADD A TIMEOUT PERIOD WITH A 1 SECOND DEFAULT VALUE
    # def download_mission(self, timeout_period: float = 1) -> mission.MissionPlan:
    def download_mission(self) -> list[mission_raw.MissionItem]:
        """
        Returns the current mission plan
        :return: mission.MissionPlan
        """
        self._logger.info("Downloading mission file")

        download_mission_task = asyncio.ensure_future(self._system.mission_raw.download_mission(), loop=self._loop)
        done_condition = Condition()
        download_mission_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        # OPTIONAL TO ADD A TIMEOUT PARAM TO REDUCE TIMEOUT ERRORS WHILE INCREASING BLOCKED THREAD TIME
        # done_condition.wait(timeout_period)

        try:
            x = download_mission_task.result()
            self._logger.info("Mission items downloaded successfully")
            return x
        except asyncio.InvalidStateError:
            # If the result is not available yet,
            #       it can be assumed that the wait call timed out before the callback was done
            self._logger.error("Could not download mission file! Request timed out!")
            return []

    def import_qgroundcontrol_mission(self, qgc_plan_path) -> mission_raw.MissionImportData:
        """
        Imports the contents from a JSON .plan format file as a mission
        :param qgc_plan_path: string ; path to a JSON .plan file
        :return: mission_raw.MissionImportData ; Returns the mission data imported
        """
        self._logger.info(f"Beginning mission import from {qgc_plan_path}")

        import_mission_task = asyncio.ensure_future(
            self._system.mission_raw.import_qgroundcontrol_mission(qgc_plan_path), loop=self._loop
        )
        done_condition = Condition()
        import_mission_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        try:
            x = import_mission_task.result()
            self._logger.info("Mission import completed successfully")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Import was not completed! Request timed out!")
            return []

    async def _update_mission_changed(self):
        """
        Updates self.has_mission_changed
        :return: None
        """
        async for mission_changed in self._system.mission_raw.mission_changed():
            if mission_changed != self.has_mission_changed:
                self.has_mission_changed = mission_changed

    def mission_changed(self) -> bool:
        """
        Notifies the user if the mission has changed
        :return: boolean ; returns True if the ground station has been uploaded or changed by a
        ground station or companion computer, False otherwise
        """
        return self.has_mission_changed

    async def _update_mission_progress(self):
        """
        Updates the current mission progress
        :return: None
        """
        async for mission_progress in self._system.mission_raw.mission_progress():
            if mission_progress != self.mission_progress:
                self.mission_progress = mission_progress

    def mission_progress(self) -> mission_raw.MissionProgress:
        """
        Get the current mission progress
        :return: mission_raw.MissionProgress ; returns the current mission progress
        """
        return self.mission_progress

    def pause_mission(self):
        """
        Pauses the current mission
        :return: None
        """
        self._logger.info("Mission Paused")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw.pause_mission(), loop=self._loop)
        )

    def set_current_mission_item(self, index: int):
        """
        Sets the current mission item to the item located at a specified index
        :param index: The index that the desired mission item is located
        :return: None
        """
        self._logger.info(f"Setting current mission to the mission at index {index}")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw.set_current_mission_item(index), loop=self._loop)
        )

    def start_mission(self):
        """
        Starts the mission that is stored in the vehicle
        :return: None
        """
        self._logger.info("Mission started")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw.start_mission(), loop=self._loop)
        )

    def upload_mission(self, items: list[mission_raw.MissionItem]):
        """
        Uploads a list of mission items to the vehicle as a mission
        :param items: list[mission_raw.MissionItem] ; List of mission items
        :return: None
        """
        self._logger.info(f"Uploaded {len(items)} mission items as a mission")
        super().submit_task(
            asyncio.ensure_future(self._system.mission_raw.upload_mission(items))
        )
