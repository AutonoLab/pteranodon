from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional, List, Callable

from mavsdk import System, mission_raw

from .abstract_base_plugin import AbstractBasePlugin


class MissionRaw(AbstractBasePlugin):
    """
    Enable raw missions as exposed by MAVLink.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("mission_raw", system, loop, logger)

        self._submit_simple_generator(self._system.mission_raw.mission_changed)
        self._submit_simple_generator(self._system.mission_raw.mission_progress)

        self._end_init()

    def cancel_mission_download(self):
        """
        Cancels the current mission download
        :return: None
        """
        self._logger.info("Canceled Mission Download")
        self._submit_coroutine(self._system.mission_raw.cancel_mission_download())

    def cancel_mission_upload(self):
        """
        cancels the current mission upload
        :return: None
        """
        self._logger.info("Canceled Mission Upload")
        self._submit_coroutine(self._system.mission_raw.cancel_mission_upload())

    def clear_mission(self):
        """
        Clears the current mission
        :return: None
        """
        self._logger.info("Cleared mission")
        self._submit_coroutine(self._system.mission_raw.clear_mission())

    def download_mission(self, timeout: float = 1.0) -> List[mission_raw.MissionItem]:
        """
        Returns the current mission plan
        :return: mission.MissionPlan
        """
        self._logger.info("Downloading mission file")

        downloaded_mission = self._submit_blocking_coroutine(
            self._system.mission_raw.download_mission(), timeout=timeout
        )

        if downloaded_mission is not None:
            self._logger.info("Mission items downloaded successfully")
            return downloaded_mission

        self._logger.error("Could not download mission file! Request timed out!")
        return []

    def import_qgroundcontrol_mission(
        self, qgc_plan_path, timeout: float = 1.0
    ) -> Optional[mission_raw.MissionImportData]:
        """
        Imports the contents from a JSON .plan format file as a mission
        :param qgc_plan_path: string ; path to a JSON .plan file
        :return: mission_raw.MissionImportData ; Returns the mission data imported
        """
        self._logger.info(f"Beginning mission import from {qgc_plan_path}")

        imported_mission = self._submit_blocking_coroutine(
            self._system.mission_raw.import_qgroundcontrol_mission(qgc_plan_path),
            timeout=timeout,
        )

        if imported_mission is not None:
            self._logger.info("Mission import completed successfully")
        else:
            self._logger.error("Import was not completed! Request timed out!")
        return imported_mission

    def mission_changed(self) -> Optional[bool]:
        """
        Notifies the user if the mission has changed
        :return: boolean ; returns True if the ground station has been uploaded or changed by a
        ground station or companion computer, False otherwise
        """
        return self._async_gen_data[self._system.mission_raw.mission_changed]

    def mission_progress(self) -> Optional[mission_raw.MissionProgress]:
        """
        Get the current mission progress
        :return: mission_raw.MissionProgress ; returns the current mission progress
        """
        return self._async_gen_data[self._system.mission_raw.mission_progress]

    def pause_mission(self):
        """
        Pauses the current mission
        :return: None
        """
        self._logger.info("Mission Paused")
        self._submit_coroutine(self._system.mission_raw.pause_mission())

    def set_current_mission_item(self, index: int):
        """
        Sets the current mission item to the item located at a specified index
        :param index: The index that the desired mission item is located
        :return: None
        """
        self._logger.info(f"Setting current mission to the mission at index {index}")
        self._submit_coroutine(self._system.mission_raw.set_current_mission_item(index))

    def start_mission(self):
        """
        Starts the mission that is stored in the vehicle
        :return: None
        """
        self._logger.info("Mission started")
        self._submit_coroutine(self._system.mission_raw.start_mission())

    def upload_mission(self, items: List[mission_raw.MissionItem]):
        """
        Uploads a list of mission items to the vehicle as a mission
        :param items: list[mission_raw.MissionItem] ; List of mission items
        :return: None
        """
        self._logger.info(f"Uploaded {len(items)} mission items as a mission")
        self._submit_coroutine(self._system.mission_raw.upload_mission(items))

    def register_mission_changed_handler(self, handler: Callable) -> None:
        """
        Registers a function (Callable) to be a handler of the data stream
        :param handler: A Callable which gets executed each time new data is received
        """
        self._register_handler(self._system.mission_raw.mission_changed)(handler)
    
    def register_mission_progress_handler(self, handler: Callable) -> None:
        """
        Registers a function (Callable) to be a handler of the data stream
        :param handler: A Callable which gets executed each time new data is received
        """
        self._register_handler(self._system.mission_raw.mission_progress)(handler)
    