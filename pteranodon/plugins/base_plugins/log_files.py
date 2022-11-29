from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import List, Optional
from functools import partial

from mavsdk import System
from mavsdk.log_files import Entry, ProgressData


from .abstract_base_plugin import AbstractBasePlugin


class LogFiles(AbstractBasePlugin):
    """
    Allow to download log files from the vehicle after a flight is complete. For log streaming during flight check the
     logging plugin.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("log_files", system, loop, logger)

        self._download_progress: Optional[ProgressData] = None
        self._entry_list: List[Entry] = []
        self._entry_list = self._loop.run_until_complete(
            self._system.log_files.get_entries()
        )

        self._end_init()

    def download_log_file(self, entry: Entry, path: str, timeout: float = 8.0) -> Optional[ProgressData]:
        """
        Download log file synchronously.

        :param entry: Entry of the log file to download.
        :type entry: Entry
        :param path: Path of where to download log file to.
        :type path: str
        :return: Download progress
        :rtype: ProgressData
        """
        self._logger.info(f"Downloading log file with id: {entry.id}")

        self._download_progress = self._submit_blocking_coroutine(
            self._system.log_files.download_log_file(entry, path),
            timeout=timeout,
        )

        if self._download_progress is None:
            self._logger.error(
                "Could not return log file download progress! Request timed out!"
            )

        return self._download_progress

    def update_entries(self) -> None:
        """
        Updates the log entries of the drone

        """
        self._submit_coroutine(
            self._system.log_files.get_entries(),
            partial(self._get_entries_callback),
        )

    def get_download_progress(self) -> Optional[ProgressData]:
        """
        Get the progress of a download

        :return: Download progress
        :rtype: ProgressData
        """
        return self._download_progress

    def erase_all_log_files(self) -> None:
        """
        Erase all log files.
        """
        self._logger.info("Erased all log files!")
        self._submit_coroutine(self._system.log_files.erase_all_log_files())

    def _get_entries_callback(self, task: Task) -> None:
        # once task is completed, store the result
        self._entry_list = task.result()

    def get_entries(self) -> List[Entry]:
        """
        Get List of log files.

        :return: Log entry list
        :rtype: List[Entry]
        """
        return self._entry_list
